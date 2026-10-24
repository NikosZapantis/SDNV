#!/usr/bin/env python3
"""fetch_map.py — build an SDNV road-network map from OpenStreetMap.

Queries the Overpass API for drivable roads (and hospitals) inside a bounding
box, converts them into a compact directed graph in local metres, keeps the
largest strongly connected component (so every vehicle can always reach every
destination) and writes a JSON file loadable by sdnv.world.Map.

Usage:
  python3 tools/fetch_map.py --bbox 37.9705,23.7220,37.9875,23.7520 \
      --name "Athens — Omonia / Syntagma" --out maps/athens.json
  python3 tools/fetch_map.py --bbox ... --classes primary,secondary,tertiary,residential
"""

import argparse
import json
import math
import sys
import urllib.parse
import urllib.request

OVERPASS = 'https://overpass-api.de/api/interpreter'
DEFAULT_CLASSES = 'trunk,primary,secondary,tertiary'
# km/h when no maxspeed tag is present (urban defaults)
CLASS_SPEED = {'motorway': 90, 'trunk': 70, 'primary': 50, 'secondary': 50, 'tertiary': 40,
               'unclassified': 30, 'residential': 30, 'living_street': 20}
CLASS_RANK = {'motorway': 5, 'trunk': 4, 'primary': 3, 'secondary': 2, 'tertiary': 1}


def overpass(bbox, classes, endpoint):
    s, w, n, e = bbox
    hw = '|'.join(classes)
    q = (f'[out:json][timeout:90];'
         f'(way["highway"~"^({hw})(_link)?$"]["area"!="yes"]({s},{w},{n},{e}););(._;>;);out body;'
         f'nwr["amenity"="hospital"]({s},{w},{n},{e});out center tags;')
    req = urllib.request.Request(endpoint, data=urllib.parse.urlencode({'data': q}).encode(),
                                 headers={'User-Agent': 'SDNV-testbed/1.0 (tools/fetch_map.py)'})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.load(r)


def parse_speed(tag, cls):
    try:
        if tag:
            v = float(tag.split()[0])
            return v * 1.609 if 'mph' in tag else v
    except ValueError:
        pass
    return CLASS_SPEED.get(cls, 30)


def largest_scc(n, adj):
    """Iterative Tarjan; returns the node set of the largest SCC."""
    index, low, on, stack, best = [None] * n, [0] * n, [False] * n, [], set()
    counter = 0
    for root in range(n):
        if index[root] is not None:
            continue
        work = [(root, 0)]
        while work:
            v, i = work.pop()
            if i == 0:
                index[v] = low[v] = counter
                counter += 1
                stack.append(v)
                on[v] = True
            recurse = False
            for j in range(i, len(adj[v])):
                w = adj[v][j]
                if index[w] is None:
                    work.append((v, j + 1))
                    work.append((w, 0))
                    recurse = True
                    break
                if on[w]:
                    low[v] = min(low[v], index[w])
            if recurse:
                continue
            if low[v] == index[v]:
                comp = set()
                while True:
                    w = stack.pop()
                    on[w] = False
                    comp.add(w)
                    if w == v:
                        break
                if len(comp) > len(best):
                    best = comp
            if work:
                u = work[-1][0]
                low[u] = min(low[u], low[v])
    return best


def simplify(pts, tol=1.5):
    """Douglas–Peucker on a list of (x, y)."""
    if len(pts) < 3:
        return pts
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    dx, dy = x2 - x1, y2 - y1
    norm = math.hypot(dx, dy) or 1e-9
    idx, dmax = 0, -1.0
    for i in range(1, len(pts) - 1):
        d = abs(dy * (pts[i][0] - x1) - dx * (pts[i][1] - y1)) / norm
        if d > dmax:
            idx, dmax = i, d
    if dmax <= tol:
        return [pts[0], pts[-1]]
    return simplify(pts[:idx + 1], tol)[:-1] + simplify(pts[idx:], tol)


def build(data, bbox, name):
    s, w, n, e = bbox
    lat0, lon0 = s, w
    kx, ky = math.cos(math.radians((s + n) / 2)) * 111320.0, 110540.0

    def xy(lat, lon):
        return ((lon - lon0) * kx, (lat - lat0) * ky)

    osm_nodes = {el['id']: (el['lat'], el['lon']) for el in data['elements'] if el['type'] == 'node' and 'lat' in el}
    ways = [el for el in data['elements'] if el['type'] == 'way' and 'nodes' in el and 'highway' in el.get('tags', {})]

    use = {}
    for wy in ways:
        nds = [x for x in wy['nodes'] if x in osm_nodes]
        for i, nd in enumerate(nds):
            use[nd] = use.get(nd, 0) + (2 if i in (0, len(nds) - 1) else 1)
    is_vertex = {nd for nd, c in use.items() if c >= 2}

    vid, verts, raw_edges, names = {}, [], [], []
    name_idx = {}

    def vertex(nd):
        if nd not in vid:
            vid[nd] = len(verts)
            verts.append(xy(*osm_nodes[nd]))
        return vid[nd]

    for wy in ways:
        tags = wy['tags']
        cls = tags['highway'].replace('_link', '')
        nds = [x for x in wy['nodes'] if x in osm_nodes]
        if len(nds) < 2:
            continue
        ow = tags.get('oneway', 'no')
        rev = ow in ('-1', 'reverse')
        one = rev or ow in ('yes', 'true', '1') or tags.get('junction') in ('roundabout', 'circular')
        fwd, bwd = not rev, rev or not one
        speed = parse_speed(tags.get('maxspeed', ''), cls)
        rname = tags.get('name:en') or tags.get('name') or tags.get('ref') or ''
        if rname not in name_idx:
            name_idx[rname] = len(names)
            names.append(rname)
        lanes = 1
        try:
            lanes = max(1, int(str(tags.get('lanes', '1')).split(';')[0]))
            if fwd and bwd:
                lanes = max(1, lanes // 2)
        except ValueError:
            pass
        seg = [nds[0]]
        for nd in nds[1:]:
            seg.append(nd)
            if nd in is_vertex:
                a, b = vertex(seg[0]), vertex(seg[-1])
                if a != b:
                    pts = [xy(*osm_nodes[x]) for x in seg]
                    meta = (round(speed / 3.6, 2), lanes, CLASS_RANK.get(cls, 0), name_idx[rname])
                    if fwd:
                        raw_edges.append((a, b, pts, meta))
                    if bwd:
                        raw_edges.append((b, a, pts[::-1], meta))
                seg = [nd]

    adj = [[] for _ in verts]
    for a, b, _p, _m in raw_edges:
        adj[a].append(b)
    keep = largest_scc(len(verts), adj)
    remap = {old: i for i, old in enumerate(sorted(keep))}
    nodes = [[round(verts[o][0], 1), round(verts[o][1], 1)] for o in sorted(keep)]
    edges, seen = [], set()
    for a, b, pts, (spd, lanes, rank, ni) in raw_edges:
        if a in keep and b in keep:
            pts = simplify(pts)
            key = (remap[a], remap[b], round(pts[len(pts) // 2][0]), round(pts[len(pts) // 2][1]))
            if key in seen:
                continue
            seen.add(key)
            mid = [[round(x, 1), round(y, 1)] for x, y in pts[1:-1]]
            edges.append([remap[a], remap[b], spd, lanes, rank, ni, mid])

    pois = []
    for el in data['elements']:
        if el.get('tags', {}).get('amenity') != 'hospital':
            continue
        c = el.get('center') or ({'lat': el['lat'], 'lon': el['lon']} if 'lat' in el else None)
        if not c:
            continue
        px, py = xy(c['lat'], c['lon'])
        best = min(range(len(nodes)), key=lambda i: (nodes[i][0] - px) ** 2 + (nodes[i][1] - py) ** 2)
        label = el['tags'].get('name:en') or el['tags'].get('name') or 'Hospital'
        if all(p['node'] != best for p in pois):
            pois.append({'type': 'hospital', 'name': label, 'node': best})

    return {
        'name': name,
        'origin': [lat0, lon0],
        'scale': [round(kx, 3), ky],
        'bbox': [s, w, n, e],
        'size': [round((e - w) * kx, 1), round((n - s) * ky, 1)],
        'names': names,
        'nodes': nodes,
        'edges': edges,
        'pois': pois,
        'attribution': '© OpenStreetMap contributors (ODbL)',
    }


def main():
    ap = argparse.ArgumentParser(description='Build an SDNV map from OpenStreetMap')
    ap.add_argument('--bbox', required=True, help='S,W,N,E in degrees')
    ap.add_argument('--out', required=True)
    ap.add_argument('--name', default='Custom area')
    ap.add_argument('--classes', default=DEFAULT_CLASSES, help=f'highway classes (default: {DEFAULT_CLASSES})')
    ap.add_argument('--endpoint', default=OVERPASS)
    ap.add_argument('--raw', help='use a cached Overpass JSON response instead of querying')
    args = ap.parse_args()

    bbox = [float(x) for x in args.bbox.split(',')]
    if len(bbox) != 4:
        sys.exit('bbox must be S,W,N,E')
    if args.raw:
        with open(args.raw) as f:
            data = json.load(f)
    else:
        print(f'[fetch_map] querying Overpass for {args.bbox} ...')
        data = overpass(bbox, args.classes.split(','), args.endpoint)
    m = build(data, bbox, args.name)
    with open(args.out, 'w') as f:
        json.dump(m, f, separators=(',', ':'), ensure_ascii=False)
    print(f'[fetch_map] {len(m["nodes"])} nodes, {len(m["edges"])} directed edges, '
          f'{len(m["pois"])} hospitals → {args.out}')


if __name__ == '__main__':
    main()

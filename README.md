# Guide-Aware Detailed Router for Standard-Cell Designs

A Python implementation of a detailed router for placed standard-cell designs. It reads **LEF**, **DEF** and global routing **GUIDE** data, maps pins/obstacles and routing tracks, routes signal nets and writes an updated DEF.

## Algorithms and implementation

- **RMST decomposition:** a Manhattan-distance Prim-style minimum spanning tree decomposes multipin nets into connection pairs.
- **Guide-aware A\* routing:** searches candidate routing-grid nodes using Manhattan-distance heuristics and costs for via changes, nonpreferred directions, leaving guides, obstacles and spacing conflicts.
- **Multi-layer routing:** extracts routing-layer directions, widths, spacing and via adjacency from technology data.
- **Physical constraints:** checks obstacles, placement boundaries, spacing and width while building shapes.
- **Connectivity repair and fallback:** includes post-routing connectivity repair and L-shaped fallback options when searches fail.
- **DEF writer:** inserts route rectangles into an output DEF.

## Requirements

- Python 3
- **`LEFDEFParser`**, the project/course parser module, with `LEFReader`, `DEFReader` and `Rect` APIs as used by the script
- Compatible placement `.def`, technology `.lef` and global routing `.guide` input files

**Important:** the external `LEFDEFParser` dependency and benchmark datasets were **not included in the uploaded materials**. This script cannot be executed end-to-end from this repository alone. Obtain and cite redistributable dependencies and datasets before claiming a reproducible run.

## Example invocation

```sh
python3 detailed_router.py -i input.def -l technology.lef -g routing.guide -o routed.def
```

or positional arguments:

```sh
python3 detailed_router.py input.def technology.lef routing.guide routed.def
```

## Evaluation plan

1. Obtain permission to use/share the parser and input benchmarks.
2. Run on a small design, validate route connectivity and check design rules with the course checker.
3. Record nets routed, opens, DRC issues, runtime and failing cases.
4. Add before/after DEF screenshots and verified checker summaries.

## Verification status

The supplied file has been preserved with only cosmetic changes to its public filename/header and usage text. Python syntax can be checked locally. The missing parser/benchmark means routing success, 100% connectivity and zero DRC are **not** claimed.

## Attribution and rights

This started as an academic detailed-routing project. Review instructor/course license and benchmark restrictions before publishing. If applicable, credit the project specification and supporting parser authors. No software license has been selected.

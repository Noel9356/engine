Photographic vegetation sprites for the planet demo's surface scatter system
(see demos/planet/source/planet_scatter.cpp). Three atlases, each a 2x2 grid of
billboard cards; every painted pixel comes from a downloaded photograph.

Sources: all plant parts are Poly Haven (https://polyhaven.com) assets
published under the CC0 1.0 Universal license (https://polyhaven.com/license)
- the same source and license as the ground/rock scans in this folder. The
composer script that built the atlases from the downloaded parts lives in the
project workspace (scripts/compose_vegetation_sprites.py) and is
deterministic: a fixed seed drives every paste, so the atlases rebuild
identically from the same source downloads.

tree_atlas.png (1024x1024, linear-radiance storage like every texture the
headless renderer samples)
    TL  broadleaf tree      island_tree_02 leaves + jacaranda bark trunk
    TR  acacia tree         jacaranda bipinnate leaves + jacaranda trunk
    BL  conifer             fir_tree_01 twig sprigs + fir bark trunk
    BR  dry broadleaf       island_tree_01 (autumn-shifted) leaves + trunk
bush_atlas.png (512x512)
    TL  green shrub         island_tree_02 leaf clusters
    TR  dry shrub           island_tree_01 leaf clusters (dry-shifted)
    BL  fern                fern_02 whole fronds, rooted at the base
    BR  twiggy scrub        fir_tree_01 twig sprigs
grass_atlas.png (512x512)
    TL  lush tuft           grass_medium_01 rooted clumps (bottom-row cutouts)
    TR  dry tuft            grass_medium_01 dry diffuse, same clumps
    BL  low sprouts         grass_bermuda_01 seedling cutouts
    BR  sparse dry clumps   grass_medium_01 dry, scattered small tufts

Processing per card (the same chain the composer applies): leaf/sprig cutouts
are extracted from the downloaded atlases by connected-component analysis of
their alpha maps, despeckled (stray unwrapping slivers and pure-white fill
texels removed, 1px scan-edge halo shaved), pasted into the card layout with
rotation and scale, then the whole card is desaturated to 45% toward its
stored-space luminance, decoded to linear, rescaled so the OPAQUE mean sits in
the living-foliage albedo band, soft-capped at the highlight shoulder, and
finally RGB-bled outward into transparent texels so minified mips blend plant
color instead of black. Fallback: when these files are missing the scatter
system generates seed-derived procedural sprites instead (same 2x2 layout),
so the demo degrades rather than vanishes.

Per-instance behavior: each scattered plant picks its species card from its
site's biome (see the species tables in planet_scatter.cpp), so different
climate zones grow visibly different flora; the vertex tint shifts hue
toward the site's ground color rather than repainting the photograph.

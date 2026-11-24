Procedural vegetation sprites for the planet demo's surface scatter system
(see demos/planet/source/planet_scatter.cpp). Three atlases, each a 2x2 grid
of billboard cards; every texel is generated deterministically by the
compositor script.

Sources: the committed compositor script
(scripts/compose_vegetation_sprites.py) generates all three atlases from a
fixed seed - a splitmix-style stream drives every blade, cluster and hole
paste, so the atlases rebuild bit-identically from the script alone. No
downloaded artwork feeds the atlases; the ground/rock scans in this folder
(see the textures README one level up) remain Poly Haven CC0 content.

tree_atlas.png (1024x1024, linear-radiance storage like every texture the
headless renderer samples)
    TL  broadleaf tree      tapered bark trunk, clustered leaf crown with a
                            vertical canopy light gradient (upper clusters
                            catch the sky, the underside only sees bounce),
                            punched canopy holes for silhouette breaks
    TR  acacia tree         lean trunk, flattened umbrella crown
    BL  conifer             barked trunk with thirteen conical tiers of
                            needle sprigs
    BR  dry broadleaf       autumn-shifted broadleaf palette
bush_atlas.png (512x512)
    TL  green shrub         mounded leaf clusters
    TR  dry shrub           dry-shifted mound
    BL  fern                nine rooted fronds, stepped leaflets
    BR  twiggy scrub        sparse twig sprigs over a low mound
grass_atlas.png (512x512)
    TL  lush tuft           110 blades fanning from a rooted base cluster
    TR  dry tuft            dry-straw palette, 80 blades
    BL  low sprouts         90 short seedling blades, wide spread
    BR  sparse dry clumps   42 small blades, wide scatter

Processing per card: every blade is a tapered two-segment stroke whose root
segment samples the tuft's self-shadowed interior (0.42-0.62 of the blade's
brightness) and whose tip segment catches full sky (1.02-1.22) - the internal
gradient is what makes a tuft read as a cluster of blades standing in shade
instead of one flat green cutout. Canopies layer clustered blobs under the
same vertical-light rule. Palettes sit in the living-foliage albedo band
pulled toward the yellow-olive a visible sward shows (a pure spectral green
reads as neon against soil). Finally each card's alpha is punched with holes
and the card is RGB-bled outward into transparent texels so minified mips
blend plant color instead of black. Fallback: when these files are missing
the scatter system generates seed-derived procedural sprites instead (same
2x2 layout), so the demo degrades rather than vanishes.

Per-instance behavior: each scattered plant picks its species card from its
site's biome (see the species tables in planet_scatter.cpp), so different
climate zones grow visibly different flora; the vertex tint shifts hue
toward the site's ground color rather than repainting the sprite.

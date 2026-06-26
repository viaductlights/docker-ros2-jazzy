"""
Color palette generator for filter-comparison RMSE plots.

Each filter FAMILY gets one base hue; each VARIANT tested within that
family gets a shade of that hue (same hue/saturation, varying lightness),
generated automatically — adding a new variant is just bumping a count,
no manual hex-picking.

Run this file directly to render a swatch preview (palette_preview.png)
before committing the colors to your real RMSE plot.
"""

import colorsys
import matplotlib.colors as mcolors


def generate_shades(base_color, n, lightness_range=(0.30, 0.70)):
    """
    Return n hex colors, all sharing base_color's hue/saturation, spread
    evenly across lightness_range (HLS lightness), darkest first.

    Varying lightness (not just blending toward white) keeps every shade
    visibly "the same color family" while staying distinguishable.
    n=1 returns the base color itself, unmodified.
    """
    r, g, b = mcolors.to_rgb(base_color)
    if n == 1:
        return [mcolors.to_hex((r, g, b))]

    h, _, s = colorsys.rgb_to_hls(r, g, b)
    lo, hi = lightness_range
    return [
        mcolors.to_hex(colorsys.hls_to_rgb(h, lo + (hi - lo) * i / (n - 1), s))
        for i in range(n)
    ]


# (base color, variant count, optional custom lightness range)
#
# Yellow gets a narrower range deliberately: pure yellow is already
# near-white at high lightness, so the default range would push a shade
# light enough to nearly disappear against a white plot background.
FILTER_SPEC = {
    'KF':              dict(color='tab:blue',  n=2),
    'EKF_landmark':    dict(color='tab:orange', n=4),
    'EKF_no_landmark': dict(color='gold',       n=2, lightness_range=(0.35, 0.55)),
    'PF':              dict(color='tab:red',    n=1),  # bump n as you add PF variants
}

FILTER_PALETTES = {
    family: generate_shades(spec['color'], spec['n'],
                             spec.get('lightness_range', (0.30, 0.70)))
    for family, spec in FILTER_SPEC.items()
}


# --- usage in your actual RMSE plot ---
#
# variant_names = {
#     'KF':              ['KF_baseline', 'KF_tuned'],
#     'EKF_landmark':    ['EKF_lm_v1', 'EKF_lm_v2', 'EKF_lm_v3', 'EKF_lm_v4'],
#     'EKF_no_landmark': ['EKF_nolm_v1', 'EKF_nolm_v2'],
#     'PF':              ['PF_v1'],
# }
#
# for family, names in variant_names.items():
#     palette = FILTER_PALETTES[family]
#     for color, name in zip(palette, names):
#         ax.plot(t, cumulative_rmse[name], color=color, label=name)


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    y = 0
    for family, palette in FILTER_PALETTES.items():
        for i, color in enumerate(palette):
            ax.barh(y, 1, color=color)
            ax.text(1.08, y, f"{family} [{i}]  {color}", va='center', fontsize=9)
            y += 1
        y += 0.6  # gap between families
    ax.set_xlim(0, 3.2)
    ax.set_ylim(-0.6, y - 0.6)
    ax.invert_yaxis()
    ax.axis('off')
    plt.tight_layout()
    plt.savefig('palette_preview.png', dpi=150)
    print('Saved palette_preview.png')

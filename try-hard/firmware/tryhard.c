/* SPDX-License-Identifier: GPL-3.0-only */
/* Try Hard: the FM-1 as a remote panel for the FM-1 emulator. Every key, button, encoder and the MASTER knob
 * goes out over USB-MIDI; the emulator page (joakimeriksson.github.io/fm1-emu) plays the firmware it runs
 * from them and sends its LEDs back. No sound here. The screen shows a clockwork of meshing gears that the
 * panel drives, over a big TRY HARD, with the link to the page under it. Protocol: ../PROTOCOL.md.
 *
 * Built on Dinghy (platform.c: boot, USB, the updater, the panel, the screen): the build drops this file in as
 * Dinghy's app (../build.sh). Integers only, no 64-bit division (Dinghy's docs/RULES.md). */
#define APP_NAME "TRY HARD"
#include "platform.c"

#define CH 0x0Fu                                /* MIDI channel 16: the panel's */
#define HELLO_CC 127u
#define HELLO_V 84u                             /* 'T' */

static void app_audio(int32_t *out, uint32_t n) { (void)out; (void)n; }   /* silent */

#ifdef DINGHY_HOST
static int32_t master_read(void) { return 700; }
#else
static int32_t master_read(void) { return fm1_adc_read(FM1_ADC_MASTER); }
#endif

/* ---------------------------------------------------------------- integer trig */
static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (dinghy.c's) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15;
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}
static int32_t sin16(uint32_t a) { return sine(a << 16); }              /* a: a turn is 65536 */
static int32_t cos16(uint32_t a) { return sine((a + 16384u) << 16); }

static const uint16_t ATAN64[65] = {           /* atan(i / 64) in 1/65536 turns */
    0, 163, 326, 489, 651, 813, 975, 1136, 1297, 1457, 1617, 1775, 1933, 2090, 2246, 2401, 2555, 2708, 2860,
    3010, 3159, 3307, 3453, 3599, 3742, 3884, 4025, 4164, 4302, 4438, 4572, 4705, 4836, 4966, 5094, 5220,
    5344, 5467, 5589, 5708, 5826, 5943, 6058, 6171, 6282, 6392, 6500, 6607, 6712, 6815, 6917, 7018, 7117,
    7214, 7310, 7405, 7498, 7589, 7679, 7768, 7856, 7942, 8026, 8110, 8192};
static uint32_t iatan2(int32_t y, int32_t x)   /* the angle of (x, y), 0..65535 (y down: clockwise) */
{
    uint32_t ax = (uint32_t)(x < 0 ? -x : x), ay = (uint32_t)(y < 0 ? -y : y), t, a;
    if (!ax && !ay)
        return 0;
    if (ax >= ay) {
        t = (ay << 10) / ax;                    /* 0..1024 */
        a = ATAN64[t >> 4] + (((ATAN64[(t >> 4) + (t < 1024u)] - ATAN64[t >> 4]) * (t & 15u)) >> 4);
    } else {
        t = (ax << 10) / ay;
        a = 16384u - (ATAN64[t >> 4] + (((ATAN64[(t >> 4) + (t < 1024u)] - ATAN64[t >> 4]) * (t & 15u)) >> 4));
    }
    if (x < 0) a = 32768u - a;
    if (y < 0) a = 65536u - a;
    return a & 0xFFFFu;
}
static uint32_t isqrt(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v) b >>= 2;
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else r >>= 1;
        b >>= 2;
    }
    return r;
}
static int32_t clamp256(int32_t v) { return v < 0 ? 0 : v > 256 ? 256 : v; }

/* ---------------------------------------------------------------- pixels (cv_px holds RGB565, byte-swapped) */
static inline void px_get(uint32_t i, int32_t *r, int32_t *g, int32_t *b)
{
    uint32_t c = swap16(cv_px[i]);
    *r = (int32_t)((c >> 11) << 3);
    *g = (int32_t)(((c >> 5) & 63u) << 2);
    *b = (int32_t)((c & 31u) << 3);
}
static inline void px_put(uint32_t i, int32_t r, int32_t g, int32_t b)
{
    r = r < 0 ? 0 : r > 255 ? 255 : r;
    g = g < 0 ? 0 : g > 255 ? 255 : g;
    b = b < 0 ? 0 : b > 255 ? 255 : b;
    cv_px[i] = swap16(RGB(r, g, b));
}

/* ---------------------------------------------------------------- the clockwork
 * Compound gears: a wheel and a pinion share an axle and turn together. Three heights (z 0..2): a pinion
 * drives the wheel of the next axle on its own height. All teeth have one module (M px), so meshing gears
 * sit their pitch radii apart (M * (N1 + N2) / 2) and their tooth phases are set so that a tooth always
 * meets a gap (gears_init). Axle a turns K[a] / 120 as fast as axle 0, the ratios of the teeth. */
#define Q4 16                                   /* positions and radii in 1/16 px */
#define M 4                                     /* module: px of pitch diameter per tooth */
#define NAX 5
#define NDISC 7
#define LIGHT 40960u                            /* light from the top left (225 degrees, y down) */
#define QWRAP (120 * 65536)                     /* the master angle wraps here: every axle turns whole */
#ifndef TIP_U
#define TIP_U 5900                              /* the tooth's tip land: 9 % of the pitch either side */
#endif
#ifndef ROOT_U
#define ROOT_U 21600                            /* its flank ends (the root begins) 33 % out */
#endif
#ifndef GAP
#define GAP 8                                   /* backlash: meshing centres this much further apart (Q4) */
#endif
#define FLANK_K ((256 << 18) / (ROOT_U - TIP_U))

typedef struct {
    uint8_t ax, z, n, spokes;                   /* axle, height, teeth, spokes (0: solid) */
    uint8_t r, g, b;                            /* the metal */
    uint8_t top;                                /* the top disc of its axle: the jewel */
    int32_t rt, rr, rim, hub;                   /* tip, root, rim inside, hub outside (Q4) */
    int32_t spk_hw, spk_k;                      /* spoke half width (Q4), 2 pi * 4096 / spokes */
} disc_t;

static disc_t disc[NDISC] = {
    /* ax z  n  sp   metal             */
    {0, 0, 18, 5, 200, 160, 76, 0, 0, 0, 0, 0, 0, 0},   /* W0: brass, the big one */
    {0, 1, 8, 0, 182, 188, 198, 0, 0, 0, 0, 0, 0, 0},   /* P0: steel pinion on W0 */
    {1, 1, 16, 5, 214, 176, 96, 0, 0, 0, 0, 0, 0, 0},   /* W1: brass, driven by P0 */
    {1, 2, 7, 0, 182, 188, 198, 0, 0, 0, 0, 0, 0, 0},   /* P1: steel pinion on W1 */
    {2, 2, 12, 4, 204, 124, 80, 0, 0, 0, 0, 0, 0, 0},   /* W2: copper, driven by P1 */
    {3, 0, 10, 3, 156, 166, 186, 0, 0, 0, 0, 0, 0, 0},  /* W3: blued steel, driven by W0 */
    {4, 2, 14, 6, 200, 160, 76, 0, 0, 0, 0, 0, 0, 0},   /* W4: brass, driven by W2 */
};
static const int32_t K[NAX] = {120, -60, 35, -216, -30};
/* the meshes, in order from axle 0: disc i drives disc j, axle j's centre in direction PHI from axle i's */
static const struct { uint8_t i, j; uint16_t phi; } MESH[NAX - 1] = {
    {1, 2, 61895},                              /* P0 -> W1, -20 degrees */
    {3, 4, 6372},                               /* P1 -> W2, +35 */
    {0, 5, 27307},                              /* W0 -> W3, 150 */
    {4, 6, 59711},                              /* W2 -> W4, -32 */
};
static int32_t ax_x[NAX], ax_y[NAX];            /* axle centres (Q4), at rest */
static uint32_t ax_c[NAX];                      /* phase of each axle */
static int32_t ax_cx[NAX], ax_cy[NAX];          /* this frame: centres with the float */
static uint32_t ax_th[NAX];                     /* this frame: angles */

static void gears_init(void)
{
    uint32_t i, k;
    ax_x[0] = 72 * Q4;
    ax_y[0] = 74 * Q4;
    ax_c[0] = 0;
    for (k = 0; k < NAX - 1u; k++) {
        const disc_t *a = &disc[MESH[k].i], *b = &disc[MESH[k].j];
        int32_t d = M * Q4 * (a->n + b->n) / 2 + GAP, num, mod = 65536 * (int32_t)b->n;
        int32_t phi = MESH[k].phi;
        ax_x[b->ax] = ax_x[a->ax] + ((d * cos16((uint32_t)phi)) >> 15);
        ax_y[b->ax] = ax_y[a->ax] + ((d * sin16((uint32_t)phi)) >> 15);
        /* tooth of a at phi meets a gap of b at phi + 180: (phi - c_a) n_a + (phi + 1/2 - c_b) n_b = 1/2 */
        num = phi * a->n + (phi + 32768) * b->n - (int32_t)ax_c[a->ax] * a->n - 32768;
        num %= mod;
        if (num < 0) num += mod;
        ax_c[b->ax] = (uint32_t)(num / b->n);
    }
    for (i = 0; i < NDISC; i++) {
        disc_t *d = &disc[i];
        int32_t pr = M * Q4 * d->n / 2;
        uint32_t j;
        d->rt = pr + M * Q4;
        d->rr = pr - (M * Q4 * 115) / 100;
        d->rim = d->rr - 4 * Q4;
        d->hub = pr * 3 / 10 < 6 * Q4 ? 6 * Q4 : pr * 3 / 10;
        d->spk_hw = 22 * Q4 / 10;
        d->spk_k = d->spokes ? 25736 / d->spokes : 0;
        d->top = 1;
        for (j = 0; j < NDISC; j++)
            if (j != i && disc[j].ax == d->ax && disc[j].z > d->z) d->top = 0;
    }
}

/* the colour of a disc at a point: 0..256 coverage, and the shaded metal into *r *g *b when r != 0.
 * px, py: the point (Q4); blur: the edge's width in px (1 sharp, more for a soft shadow). */
static int32_t disc_at(const disc_t *d, int32_t px, int32_t py, int32_t blur, int32_t *cr, int32_t *cg, int32_t *cb)
{
    int32_t dx = px - ax_cx[d->ax], dy = py - ax_cy[d->ax], pad = (blur + 1) * Q4 / 2, kq = 256 / blur;
    int32_t r, su, us, prof, edge, cov, cut = 0, L, nrm, bev = 0, t = 0, sv = 0;
    uint32_t a, u;
    if (dx > d->rt + pad || dx < -d->rt - pad || dy > d->rt + pad || dy < -d->rt - pad)
        return 0;
    r = (int32_t)isqrt((uint32_t)(dx * dx + dy * dy));
    if (r > d->rt + pad)
        return 0;
    a = iatan2(dy, dx);
    /* the teeth: u is the place within a tooth, 0 at its middle; a trapezoid with rounded corners */
    u = ((a - ax_th[d->ax]) * d->n) & 0xFFFFu;
    su = (int16_t)u;
    us = su < 0 ? -su : su;
    if (us < TIP_U) prof = 256;                 /* the tip land */
    else if (us > ROOT_U) prof = 0;             /* the root */
    else {
        prof = ((ROOT_U - us) * FLANK_K) >> 18; /* the flank */
        prof = (prof * prof * (768 - 2 * prof)) >> 16;   /* smoothstep: rounded */
    }
    edge = d->rr + (((d->rt - d->rr) * prof) >> 8);
    cov = clamp256(128 + (((edge - r) * kq) >> 4));
    if (!cov)
        return 0;
    if (r < 2 * Q4 + pad && !d->top)            /* the axle hole (the top disc has the jewel) */
        cut = clamp256(128 + (((2 * Q4 - r) * kq) >> 4));
    if (d->spokes && r > d->hub - pad && r < d->rim + pad) {   /* the windows between the spokes */
        int32_t ang, w;
        sv = (int16_t)(((a - ax_th[d->ax]) * d->spokes) & 0xFFFFu);
        ang = ((sv < 0 ? -sv : sv) * d->spk_k) >> 16;          /* radians, Q12 */
        t = (r * ang) >> 12;                    /* px from the spoke's middle line (Q4) */
        w = clamp256(128 + (((t - d->spk_hw) * kq) >> 4));
        if (w > clamp256(128 + (((r - d->hub) * kq) >> 4))) w = clamp256(128 + (((r - d->hub) * kq) >> 4));
        if (w > clamp256(128 + (((d->rim - r) * kq) >> 4))) w = clamp256(128 + (((d->rim - r) * kq) >> 4));
        if (w > cut) cut = w;
    }
    cov = (cov * (256 - cut)) >> 8;
    if (!cr || !cov)
        return cov;

    if (d->top && r < 11 * Q4 / 2) {            /* the jewel: a ruby in a gold setting */
        if (r < 3 * Q4) {
            int32_t hx = dx + Q4, hy = dy + Q4, h = hx * hx + hy * hy, l = 256 - (r * 40 >> 4);
            *cr = 190 * l >> 8; *cg = 22 * l >> 8; *cb = 48 * l >> 8;
            if (h < 20 * Q4 * Q4 / 10) { *cr = 255; *cg = 214; *cb = 222; }
        } else {
            L = 220 + ((cos16(a - LIGHT) * 80) >> 15);
            *cr = 226 * L >> 8; *cg = 184 * L >> 8; *cb = 92 * L >> 8;
        }
        return cov;
    }
    /* brushed metal: a conic sheen that stays with the light while the disc turns */
    L = 196 + ((cos16(2u * (a - LIGHT)) * 44) >> 15);
    nrm = -1;
    if (r < d->hub) {                           /* the hub: raised, turned rings */
        L += 22;
        if (((r >> 4) % 3) == 0) L -= 14;
        if (d->hub - r < 3 * Q4 / 2) { nrm = (int32_t)a; bev = 150; }
        if (r < 3 * Q4) { nrm = (int32_t)(a + 32768u); bev = 120; }   /* down into the axle hole */
    } else if (edge - r < 2 * Q4) {             /* the outer edge: the teeth's chamfer */
        nrm = (int32_t)a;
        if (prof > 8 && prof < 248)
            nrm = (int32_t)(a + (uint32_t)(su > 0 ? 12000 : -12000));   /* a flank faces sideways */
        bev = 130;
    } else if (d->spokes && r > d->hub) {      /* a window's edges: chamfers down into it */
        if (r < d->hub + 3 * Q4 / 2 && t > d->spk_hw) { nrm = (int32_t)a; bev = 120; }
        else if (r > d->rim - 3 * Q4 / 2 && r <= d->rim + Q4 / 2 && t > d->spk_hw) { nrm = (int32_t)(a + 32768u); bev = 120; }
        else if (r < d->rim && t > d->spk_hw - 3 * Q4 / 2 && t <= d->spk_hw + Q4 / 2)
            { nrm = (int32_t)(a + (uint32_t)(sv > 0 ? 16384 : -16384)); bev = 120; }
        if (r > d->rim) L -= 10;                /* the rim: a step lower than the teeth's face */
    } else if (!d->spokes && r < d->hub + 3 * Q4 / 2 && r > d->hub) {
        L -= 30;                                /* a pinion: the groove around its hub */
    }
    if (nrm >= 0) {
        int32_t lam = cos16((uint32_t)nrm - LIGHT);   /* -32767..32767 */
        L += (lam * bev) >> 15;
        if (lam > 0) L += (((lam * lam) >> 15) * 70) >> 15;   /* the glint */
    }
    if (L < 40) L = 40;
    *cr = d->r * L >> 8;
    *cg = d->g * L >> 8;
    *cb = d->b * L >> 8;
    return cov;
}

/* ---------------------------------------------------------------- text, scaled and blended */
static void glyph_draw(int32_t x, int32_t y, uint32_t gi, int32_t s, int32_t y0, int32_t y1,
                       int32_t cr, int32_t cg, int32_t cb, int32_t alpha)
{   /* glyph gi of FONT_L (or FONT_S when s < 0, unscaled) at (x, y), scale s/256, into strip rows y0..y1 */
    const felucca_font_t *f = s < 0 ? &FONT_S : &FONT_L;
    int32_t sc = s < 0 ? 256 : s, inv = 65536 / sc;
    int32_t w = f->bw[gi], h = f->h, bpr = (w + 1) / 2, dw = (w * sc) >> 8, dh = (h * sc) >> 8, X, Y;
    const uint8_t *gd = f->data + f->off[gi];
    x -= (f->pad * sc) >> 8;
    for (Y = y < y0 ? y0 : y; Y < y + dh + 1 && Y < y1; Y++) {
        int32_t fy = (Y - y) * inv + inv / 2 - 128, gy = fy >> 8, ty = fy & 255;
        for (X = x < 0 ? 0 : x; X < x + dw + 1 && X < 240; X++) {
            int32_t fx = (X - x) * inv + inv / 2 - 128, gx = fx >> 8, tx = fx & 255, v[4], k, a;
            for (k = 0; k < 4; k++) {
                int32_t xx = gx + (k & 1), yy = gy + (k >> 1), b;
                if (xx < 0 || yy < 0 || xx >= w || yy >= h) { v[k] = 0; continue; }
                b = gd[yy * bpr + xx / 2];
                v[k] = (xx & 1) ? (b & 15) : (b >> 4);
            }
            if (fx < 0) { tx = 0; } if (fy < 0) { ty = 0; }
            a = ((v[0] * (256 - tx) + v[1] * tx) * (256 - ty) + (v[2] * (256 - tx) + v[3] * tx) * ty) >> 16;   /* 0..15 */
            a = (a * 273 >> 4) * alpha >> 8;
            if (a > 0) {
                uint32_t i = (uint32_t)(Y - y0) * 240u + (uint32_t)X;
                int32_t r, g, b;
                px_get(i, &r, &g, &b);
                px_put(i, r + (((cr - r) * a) >> 8), g + (((cg - g) * a) >> 8), b + (((cb - b) * a) >> 8));
            }
        }
    }
}
static void text_small(int32_t x, int32_t y, const char *s, int32_t cr, int32_t cg, int32_t cb, int32_t y0, int32_t y1)
{
    if (y >= y1 || y + 16 <= y0) return;
    for (; *s; s++) {
        uint32_t gi = glyph(&FONT_S, (uint8_t)*s);
        glyph_draw(x, y, gi, -1, y0, y1, cr, cg, cb, 256);
        x += FONT_S.adv[gi];
    }
}

/* ---------------------------------------------------------------- the screen, a strip at a time */
#define STRIP 48
static const char TITLE[] = "TRY HARD";
static const uint8_t TCOL[8][3] = {
    {255, 214, 64}, {255, 156, 52}, {255, 96, 140}, {0, 0, 0}, {178, 124, 255}, {72, 212, 232},
    {124, 232, 124}, {255, 214, 64}};
static int32_t title_s, title_x, wave;          /* scale, left edge, how much the letters jump */
static int32_t th_kick;                         /* speed the panel gave the gears since the last frame */
static uint32_t Qm;                             /* the master angle: axle 0 at Qm / 120 */
static int32_t lift;                            /* the float height above the ground (Q4) */
static char line1[40], line2[40];
static int linked;

static void frame_setup(uint32_t now)
{
    static uint32_t last;
    static int32_t speed_x;                     /* extra speed from playing, decays */
    uint32_t dt = now - last, a;
    int32_t bob;
    if (dt > 200u) dt = 200u;
    last = now;
    speed_x += th_kick;
    th_kick = 0;
    Qm = (uint32_t)(((int32_t)Qm + ((linked ? 400 : 160) + speed_x) * (int32_t)dt) % QWRAP + QWRAP) % QWRAP;
    speed_x -= (speed_x * (int32_t)dt) >> 9;
    if (wave > 0) wave -= (int32_t)dt / 2 + 1;
    if (wave < 0) wave = 0;
    bob = (sine(now * 1431655u) * 3 * Q4) >> 15;          /* +-3 px, every 3 s */
    lift = 8 * Q4 + bob;
    for (a = 0; a < NAX; a++) {
        ax_cx[a] = ax_x[a];
        ax_cy[a] = ax_y[a] - bob;
        ax_th[a] = (uint32_t)((int32_t)Qm / 120 * K[a] + (int32_t)(Qm % 120u) * K[a] / 120) + ax_c[a];
        ax_th[a] &= 0xFFFFu;
    }
}

static void strip_draw(uint32_t s, uint32_t now)
{
    int32_t y0 = (int32_t)s * STRIP, y1 = y0 + STRIP, x, y, i;
    cv_begin(240, STRIP, C_BLACK);
    for (y = y0; y < y1; y++) {                 /* the ground: deep blue to plum */
        int32_t r = 16 + y * 22 / 240, g = 18 + y * 4 / 240, b = 34 + y * 22 / 240;
        for (x = 0; x < 240; x++) {
            int32_t v = (x - 120) * (x - 120) / 900;           /* a little vignette */
            px_put((uint32_t)(y - y0) * 240u + (uint32_t)x, r - v, g - v, b - v);
        }
    }
    /* TRY HARD: big, each letter bobbing on its own, a drop shadow under it */
    {
        int32_t px0 = title_x, k;
        for (k = 0; TITLE[k]; k++) {
            uint32_t gi = glyph(&FONT_L, (uint8_t)TITLE[k]);
            int32_t adv = (FONT_L.adv[gi] * title_s) >> 8;
            int32_t ly = 150 + ((sine(now * 715828u + (uint32_t)k * 600000000u) * (4 + wave / 16)) >> 15);
            int32_t dh = (FONT_L.h * title_s) >> 8;
            if (TITLE[k] != ' ' && ly - 2 < y1 && ly + dh + 6 > y0) {
                glyph_draw(px0 + 3, ly + 4, gi, title_s, y0, y1, 6, 4, 14, 170);
                glyph_draw(px0, ly, gi, title_s, y0, y1, TCOL[k][0], TCOL[k][1], TCOL[k][2], 256);
            }
            px0 += adv;
        }
    }
    /* the link, and what went out last */
    {
        int32_t on = linked || ((now / 500u) & 1u);
        if (y0 < 230 && y1 > 202) {
            int32_t cx = 14, cy = 213, rr;
            for (y = y0 < 206 ? 206 : y0; y < 221 && y < y1; y++)
                for (x = 8; x < 21; x++) {
                    rr = (x - cx) * (x - cx) + (y - cy) * (y - cy);
                    if (rr <= 20 && on)
                        px_put((uint32_t)(y - y0) * 240u + (uint32_t)x, linked ? 90 : 255, linked ? 230 : 170, linked ? 110 : 40);
                }
        }
        text_small(26, 205, line1, linked ? 200 : 230, linked ? 240 : 200, linked ? 210 : 150, y0, y1);
        text_small(26, 222, line2, 130, 120, 160, y0, y1);
    }
    /* the clockwork: shadows on the ground, then the discs bottom up, each shaded by those above it */
    for (y = y0; y < y1; y++) {
        int32_t py = y * Q4 + Q4 / 2;
        if (y > 150) break;
        for (x = 0; x < 240; x++) {
            int32_t px = x * Q4 + Q4 / 2, r, g, b, sh = 0, z, any = 0;
            uint32_t idx = (uint32_t)(y - y0) * 240u + (uint32_t)x;
            for (i = 0; i < NDISC; i++) {       /* the ground: the light comes from the top left */
                int32_t h = lift + disc[i].z * 3 * Q4, c;
                c = disc_at(&disc[i], px - h * 6 / 10, py - h * 8 / 10, 3 + h / (4 * Q4), 0, 0, 0);
                if (c) { sh = sh + c - ((sh * c) >> 8); any = 1; }
            }
            for (i = 0; i < NDISC && !any; i++)
                if (disc_at(&disc[i], px, py, 1, 0, 0, 0)) any = 1;
            if (!any)
                continue;
            px_get(idx, &r, &g, &b);
            r -= (r * sh * 150) >> 16; g -= (g * sh * 150) >> 16; b -= (b * sh * 150) >> 16;
            for (z = 0; z < 3; z++)
                for (i = 0; i < NDISC; i++) {
                    int32_t dr, dg, db, c, j, s2 = 0;
                    if (disc[i].z != z) continue;
                    c = disc_at(&disc[i], px, py, 1, &dr, &dg, &db);
                    if (!c) continue;
                    for (j = 0; j < NDISC; j++) {           /* shadows of the discs above */
                        int32_t h = (disc[j].z - z) * 3 * Q4, cj;
                        if (disc[j].z <= z) continue;
                        cj = disc_at(&disc[j], px - h * 6 / 10, py - h * 8 / 10, 2, 0, 0, 0);
                        if (cj) s2 = s2 + cj - ((s2 * cj) >> 8);
                    }
                    dr -= (dr * s2 * 120) >> 16; dg -= (dg * s2 * 120) >> 16; db -= (db * s2 * 120) >> 16;
                    r += ((dr - r) * c) >> 8; g += ((dg - g) * c) >> 8; b += ((db - b) * c) >> 8;
                }
            px_put(idx, r, g, b);
        }
    }
    cv_blit(0, (uint32_t)y0);
}

/* ---------------------------------------------------------------- what the panel did, in words */
static const char *const BTN_NAME[14] = {"OCT-", "OCT+", "FX", "SEL", "ENV", "LFO", "EDIT", "GLO", "HOME", "SAVE",
                                         "ARP", "SEQ", "PLAY", "REC"};
static const char *const ENC_NAME[7] = {"SELECT", "ALGORITHM", "KNOB1", "KNOB2", "KNOB3", "KNOB4", "PRESETS"};
static char *put_s(char *p, const char *s) { while (*s) *p++ = *s++; *p = 0; return p; }
static char *put_u(char *p, uint32_t v)
{
    char t[10];
    int n = 0;
    do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) *p++ = t[--n];
    *p = 0;
    return p;
}
static uint32_t sent;
static void said(uint32_t id, int32_t v, int kind)  /* kind 0 key/button, 1 encoder, 2 MASTER */
{
    static const char *const NN[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    char *p = line2;
    if (kind == 0 && id >= 14u) {
        uint32_t note = 53u + id - 14u;         /* key 0 = F3 */
        p = put_s(p, "KEY ");
        p = put_s(p, NN[note % 12u]);
        p = put_u(p, note / 12u - 1u);
    } else if (kind == 0) {
        p = put_s(p, BTN_NAME[id]);
    } else if (kind == 1) {
        p = put_s(p, ENC_NAME[id]);
        p = put_s(p, v < 0 ? " -" : " +");
        p = put_u(p, (uint32_t)(v < 0 ? -v : v));
    } else {
        p = put_s(p, "MASTER ");
        p = put_u(p, (uint32_t)v);
    }
    p = put_s(p, "  #");
    put_u(p, ++sent);
}

/* ---------------------------------------------------------------- the panel out, the LEDs in */
static uint8_t led_want[41], led_set[41];
static uint32_t last_page, last_hello;
static int32_t master_sent = -1000, master_avg = -1;

static void send_master(int32_t v)
{
    midi_send(0xB0u | CH, 7, (uint32_t)v >> 3);
    midi_send(0xB0u | CH, 39, ((uint32_t)v & 7u) << 4);
    master_sent = v;
}

static void panel_out(void)
{
    static uint32_t prev, t_master;
    uint32_t keys = fm1_in.notes, pressed = fm1_input_note_edges(), b, rel, k;
    int32_t v;
    for (k = 0; k < 27u; k++) {
        uint32_t m = 1u << k, id = 14u + k;
        if (pressed & m) {
            if (prev & m) midi_send(0x80u | CH, id, 0);
            midi_send(0x90u | CH, id, 127);
            said(id, 0, 0);
            th_kick += 300;
            wave = 256;
        } else if ((keys & m) && !(prev & m)) {
            midi_send(0x90u | CH, id, 127);
            said(id, 0, 0);
            th_kick += 300;
            wave = 256;
        }
        if (!(keys & m) && ((prev & m) || (pressed & m)))
            midi_send(0x80u | CH, id, 0);
    }
    prev = keys;
    b = fm1_input_edges(&rel);
    for (k = 0; k < 14u; k++) {
        uint32_t m = 1u << k;
        if (b & m) { midi_send(0x90u | CH, k, 127); said(k, 0, 0); th_kick += 300; wave = 256; }
        if (rel & m) {
            midi_send(0x80u | CH, k, 0);
            if (fm1_in.buttons & m) midi_send(0x90u | CH, k, 127);
        }
    }
    for (k = 0; k < 7u; k++) {
        int32_t d = fm1_enc_take(k);
        if (!d) continue;
        v = 64 + d;
        midi_send(0xB0u | CH, 16u + k, (uint32_t)(v < 1 ? 1 : v > 127 ? 127 : v));
        said(k, d, 1);
        th_kick += d * 900;                     /* a knob turns the gears, either way */
    }
    if ((v = master_read()) >= 0) {
        master_avg = master_avg < 0 ? v * 16 : master_avg + (v * 16 - master_avg) / 4;
        v = master_avg / 16;
        if ((v - master_sent > 3 || master_sent - v > 3 || (v != master_sent && (v == 0 || v == 1023)))
            && fm1_ms - t_master >= 10u) {
            t_master = fm1_ms;
            send_master(v);
            said(0, v, 2);
        }
    }
    if (fm1_ms - last_hello >= 1000u) {          /* here I am */
        last_hello = fm1_ms;
        midi_send(0xB0u | CH, HELLO_CC, HELLO_V);
    }
}

static void midi_in(void)
{
    uint8_t st, d1, d2;
    uint32_t k;
    while (midi_get(&st, &d1, &d2)) {
        if ((st & 0x0Fu) != CH) continue;
        if ((st & 0xF0u) == 0x90u && d1 < 41u) led_want[d1] = d2 ? 1u : 0u;
        else if ((st & 0xF0u) == 0x80u && d1 < 41u) led_want[d1] = 0;
        else if ((st & 0xF0u) == 0xB0u && d1 == HELLO_CC) {
            if (!linked) {                      /* a page came: tell it where MASTER is */
                linked = 1;
                if (master_sent >= 0) send_master(master_sent);
            }
            last_page = fm1_ms;
        } else if ((st & 0xF0u) == 0xB0u && d1 == 123u)
            for (k = 0; k < 41u; k++) led_want[k] = 0;
    }
    if (linked && fm1_ms - last_page > 3000u) {  /* the page went away */
        linked = 0;
        for (k = 0; k < 41u; k++) led_want[k] = 0;
    }
}

static void leds(void)
{
    uint32_t k, chase = (fm1_ms / 90u) % 40u;
    for (k = 0; k < 41u; k++) {
        uint8_t on = led_want[k];
        if (!linked) on = k >= 14u && (k - 14u == chase || k - 14u + 1u == chase);   /* waiting: a chase */
        if (on != led_set[k]) { led_set[k] = on; fm1_led_key(k, on); }
    }
}

static void app_main(void)
{
    uint32_t s = 0;
    gears_init();
    title_s = (226 * 256) / text_w(&FONT_L, TITLE);
    if (title_s > 2 * 256) title_s = 2 * 256;
    title_x = (240 - ((text_w(&FONT_L, TITLE) * title_s) >> 8)) / 2;
    put_s(line2, "usb-midi channel 16");
    for (;;) {
        platform_poll();
        panel_out();
        midi_in();
        leds();
        put_s(line1, linked ? "linked to the emulator" : "waiting for the emulator page");
        if (s == 0) frame_setup(fm1_ms);
        strip_draw(s, fm1_ms);
        if (++s * STRIP >= 240u) s = 0;
    }
}

"""
espacio_trabajo.py: Análisis del espacio de trabajo del brazo para la tarea de repostaje.

Pregunta 1: ¿qué posiciones de la boca son alcanzables con la orientación requerida?
    Para cada posición de la boca en una malla (x, distancia lateral, altura), y en cada carril,
    se resuelve la IK del vía point y del punto de inserción con la boquilla alineada con el eje
    del cuello, y se recorre el tramo lineal entre ambos. La posición es alcanzable si la IK
    converge en todo el tramo, partiendo de la misma semilla que usa el ciclo.

Pregunta 2: ¿qué tan cerca de una singularidad queda el brazo?
    Se usa el índice de manipulabilidad de Yoshikawa, w = sqrt(det(J J^T)) = |det J|, con el
    jacobiano de modelo_dh.py. Para la geometría del UR5 el determinante se factoriza en
        det J = a2 a3 · sin q3 · sin q5 · (a2 cos q2 + a3 cos(q2+q3) + d5 sin(q2+q3+q4))
    y cada factor se anula en una singularidad distinta: codo (q3 = 0, brazo estirado),
    muñeca (q5 = 0, ejes 4 y 6 alineados) y hombro (el centro de la muñeca sobre el eje 1).
    Se reporta w normalizado con su máximo para el UR5 (w_max = 0,1199, en q3 = 72,3°).

Uso:
    python espacio_trabajo.py
Salidas:
    resultados/espacio_trabajo.csv   una fila por punto de la malla
    resultados/espacio_tipos.csv     una fila por tipo de vehículo (peor caso en su zona de estacionamiento)
    resultados/espacio_trabajo.png   mapas de manipulabilidad con los cuatro tipos marcados
"""
import csv
import os
import numpy as np

import modelo_dh as m
import escenario as esc

A2, A3, D5 = m.DH_TABLA[1][0], m.DH_TABLA[2][0], m.DH_TABLA[4][2]
MUESTRAS_TRAMO = 10


def _w_max():
    """Máximo de |det J| del UR5. Con q2 y q4 libres, el factor del hombro llega a
    sqrt(a2² + a3² + 2 a2 a3 cos q3) + d5, así que solo queda maximizar sobre q3."""
    q3 = np.linspace(0, np.pi, 200001)
    w = abs(A2 * A3) * np.sin(q3) * (np.sqrt(A2 ** 2 + A3 ** 2 + 2 * A2 * A3 * np.cos(q3)) + D5)
    return float(w.max())


W_MAX = _w_max()


def factores_singularidad(q):
    """(|sin q3|, |sin q5|, |factor del hombro| [m]). El menor indica la singularidad más cercana."""
    _, q2, q3, q4, q5, _ = q
    hombro = A2 * np.cos(q2) + A3 * np.cos(q2 + q3) + D5 * np.sin(q2 + q3 + q4)
    return abs(np.sin(q3)), abs(np.sin(q5)), abs(hombro)


def evaluar_boca(p_boca, lado):
    """Evalúa la maniobra completa para una boca en p_boca (mundo) en el carril 'lado'.
    Devuelve None si no es alcanzable, o un dict con la manipulabilidad y los factores."""
    eje = esc.eje_boca(lado)
    p_aprox = esc.a_base(p_boca - esc.DIST_APROXIMACION * eje)
    p_inser = esc.a_base(p_boca + esc.PROFUNDIDAD_INSERCION * eje)
    q, ok, _ = m.ik(p_aprox, eje, esc.q_semilla(lado))
    if not ok:
        return None
    ws, qs = [m.manipulabilidad(q)], [q]
    for k in range(1, MUESTRAS_TRAMO + 1):
        p = p_aprox + (k / MUESTRAS_TRAMO) * (p_inser - p_aprox)
        q, ok, _ = m.ik(p, eje, q)
        if not ok:
            return None
        ws.append(m.manipulabilidad(q))
        qs.append(q)
    # los factores se reportan en el punto del tramo más cercano a una singularidad
    k_peor = int(np.argmin(ws))
    codo, muneca, hombro = factores_singularidad(qs[k_peor])
    J = m.jacobiano(qs[k_peor])
    return dict(w_insercion=ws[-1] / W_MAX, w_min_tramo=ws[k_peor] / W_MAX,
                avance_peor=k_peor / MUESTRAS_TRAMO,
                sigma_min=float(np.linalg.svd(J, compute_uv=False)[-1]),
                sen_q3=codo, sen_q5=muneca, hombro_m=hombro)


def punto(x, dist_lateral, z, lado):
    return np.array([x, lado * dist_lateral, z])


def malla(xs, ls, zs, lados=(-1, +1)):
    filas = []
    for lado in lados:
        for x in xs:
            for L in ls:
                for z in zs:
                    r = evaluar_boca(punto(x, L, z, lado), lado)
                    filas.append(dict(lado=lado, x=round(x, 3), dist_lateral=round(L, 3), z=round(z, 3),
                                      alcanzable=r is not None, **(r or {})))
    return filas


def zona_tipo(tipo, n=3):
    """Peor caso dentro de la zona de estacionamiento del tipo (dx ±10 cm, dy ±5 cm)."""
    lado = esc.lado_de(tipo)
    res = []
    for dx in np.linspace(-esc.RANGO_DX, esc.RANGO_DX, n):
        for dy in np.linspace(-esc.RANGO_DY, esc.RANGO_DY, n):
            res.append(evaluar_boca(esc.posicion_boca(dx, dy, tipo), lado))
    ok = [r for r in res if r is not None]
    fila = dict(tipo=tipo, boca=esc.TIPOS_VEHICULO[tipo]["boca"], carril=lado,
                altura_boca_m=esc.TIPOS_VEHICULO[tipo]["altura_boca"],
                puntos_alcanzables=f"{len(ok)}/{len(res)}")
    if ok:
        fila.update(w_insercion_min=min(r["w_insercion"] for r in ok),
                    w_insercion_max=max(r["w_insercion"] for r in ok),
                    w_tramo_min=min(r["w_min_tramo"] for r in ok),
                    sen_q3_min=min(r["sen_q3"] for r in ok),
                    sen_q5_min=min(r["sen_q5"] for r in ok),
                    hombro_min_m=min(r["hombro_m"] for r in ok))
    return fila


def rango_continuo(valores, alcanzable, centro):
    """Tramo continuo de valores alcanzables que contiene a 'centro'."""
    i = int(np.argmin(np.abs(np.asarray(valores) - centro)))
    if not alcanzable[i]:
        return None
    a = b = i
    while a > 0 and alcanzable[a - 1]:
        a -= 1
    while b < len(valores) - 1 and alcanzable[b + 1]:
        b += 1
    return valores[a], valores[b]


def texto_rango(r, malla_vals):
    a = ("<= " if r[0] == malla_vals[0] else "") + f"{r[0]:.3f}"
    b = (">= " if r[1] == malla_vals[-1] else "") + f"{r[1]:.3f}"
    return f"{a} a {b}"


def guardar(ruta, filas):
    claves = list(dict.fromkeys(k for f in filas for k in f))
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=claves)
        w.writeheader()
        for fila in filas:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in fila.items()})


def figura(cortes, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cmap = plt.get_cmap("Blues").copy()
    cmap.set_bad("#e6e6e6")
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.6), dpi=130, constrained_layout=True)
    for ax, (titulo, ejex, xs, zs, W, marcas) in zip(axs, cortes):
        im = ax.pcolormesh(xs, zs, np.ma.masked_invalid(W), cmap=cmap, vmin=0, vmax=1, shading="nearest")
        ax.contour(xs, zs, np.nan_to_num(W, nan=0.0), levels=[0.25], colors="#444", linewidths=1,
                   linestyles="--")
        for etiqueta, x0, x1, z, dx_etq in marcas:
            ax.add_patch(plt.Rectangle((x0, z - 0.004), x1 - x0, 0.008, color="#c0392b", zorder=5))
            ax.annotate(etiqueta, ((x0 + x1) / 2, z), xytext=(dx_etq, 6), textcoords="offset points",
                        fontsize=8, color="#222", zorder=6)
        ax.set_title(titulo, fontsize=10)
        ax.set_xlabel(ejex)
        ax.set_ylabel("altura de la boca z [m]")
    cb = fig.colorbar(im, ax=axs, shrink=0.9)
    cb.set_label("manipulabilidad en la inserción  w / w_max")
    fig.text(0.01, -0.03, "Gris: no alcanzable con la boquilla alineada al cuello. "
             "Línea discontinua: w / w_max = 0,25. Barras rojas: zona de estacionamiento de cada tipo.",
             fontsize=8, color="#444")
    fig.savefig(ruta, bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs("resultados", exist_ok=True)
    print(f"w_max del UR5 = {W_MAX:.4f}")

    # ---- malla gruesa en 3D, los dos carriles
    xs = np.round(np.arange(-0.30, 0.301, 0.10), 3)
    ls = np.round(np.arange(0.40, 1.101, 0.05), 3)
    zs = np.round(np.arange(0.50, 1.501, 0.05), 3)
    filas = malla(xs, ls, zs)
    guardar("resultados/espacio_trabajo.csv", filas)
    por_lado = {lado: {(f["x"], f["dist_lateral"], f["z"]): f for f in filas if f["lado"] == lado}
                for lado in (-1, +1)}
    n_ok = {lado: sum(f["alcanzable"] for f in d.values()) for lado, d in por_lado.items()}
    # el carril +1 es el -1 girado 180° alrededor del eje 1 del robot: (x, y) -> (-x, -y)
    espejo = {k: por_lado[+1][(-k[0] + 0.0, k[1], k[2])] for k in por_lado[-1]}
    dif_alc = sum(por_lado[-1][k]["alcanzable"] != espejo[k]["alcanzable"] for k in por_lado[-1])
    dif_w = max((abs(por_lado[-1][k]["w_insercion"] - espejo[k]["w_insercion"])
                 for k in por_lado[-1] if por_lado[-1][k]["alcanzable"] and espejo[k]["alcanzable"]),
                default=0.0)
    print(f"Malla: {len(filas) // 2} posiciones por carril; alcanzables {n_ok[-1]} (carril -1) "
          f"y {n_ok[+1]} (carril +1)")
    print(f"Simetría entre carriles (x del carril +1 invertido): {dif_alc} posiciones con distinta "
          f"alcanzabilidad, diferencia máxima de w/w_max {dif_w:.1e}")

    # ---- cortes finos para la figura (carril -1; el +1 es su espejo)
    lf = np.round(np.arange(0.40, 1.101, 0.025), 3)
    zf = np.round(np.arange(0.50, 1.501, 0.025), 3)
    xf = np.round(np.arange(-0.70, 0.701, 0.025), 3)
    W1 = np.full((len(zf), len(lf)), np.nan)
    for j, L in enumerate(lf):
        for i, z in enumerate(zf):
            r = evaluar_boca(punto(0.0, L, z, -1), -1)
            W1[i, j] = r["w_insercion"] if r else np.nan
    W2 = np.full((len(zf), len(xf)), np.nan)
    for j, x in enumerate(xf):
        for i, z in enumerate(zf):
            r = evaluar_boca(punto(x, esc.DIST_LATERAL_NOMINAL, z, -1), -1)
            W2[i, j] = r["w_insercion"] if r else np.nan
    L0, R = esc.DIST_LATERAL_NOMINAL, esc.TIPOS_VEHICULO
    desp = {"compacto": -60, "sedan": 8, "suv": -40, "pickup": 8}
    marcas1 = [(t, L0 - esc.RANGO_DY, L0 + esc.RANGO_DY, R[t]["altura_boca"], desp[t]) for t in esc.TIPOS]
    marcas2 = [(t, -esc.RANGO_DX, esc.RANGO_DX, R[t]["altura_boca"], desp[t]) for t in esc.TIPOS]
    figura([("Carril -1: corte en x = 0 (boca frente al robot)", "distancia lateral de la boca a la base [m]",
             lf, zf, W1, marcas1),
            (f"Carril -1: corte a {L0:.2f}".replace(".", ",") + " m de distancia lateral (en el +1, x cambia de signo)", "posición longitudinal de la boca x [m]",
             xf, zf, W2, marcas2)],
           "resultados/espacio_trabajo.png")

    # ---- rangos alcanzables alrededor de la posición nominal
    print("\nAlrededor de la posición nominal (x = 0, distancia lateral 0,70 m):")
    j0 = int(np.argmin(np.abs(lf - L0)))
    rz = rango_continuo(list(zf), list(~np.isnan(W1[:, j0])), 1.0)
    print(f"  alturas de boca alcanzables: {texto_rango(rz, zf)} m")
    for t in esc.TIPOS:
        i0 = int(np.argmin(np.abs(zf - R[t]["altura_boca"])))
        rl = rango_continuo(list(lf), list(~np.isnan(W1[i0, :])), L0)
        rx = rango_continuo(list(xf), list(~np.isnan(W2[i0, :])), 0.0)
        print(f"  {t:<9} (z = {R[t]['altura_boca']:.2f} m): distancia lateral {texto_rango(rl, lf)} m, "
              f"x {texto_rango(rx, xf)} m")
    print("  (<= o >=: el rango llega al borde de la malla evaluada)")

    # ---- cada tipo en su zona de estacionamiento
    tipos = [zona_tipo(t) for t in esc.TIPOS]
    guardar("resultados/espacio_tipos.csv", tipos)
    print(f"\n{'tipo':<9} {'boca':<10} {'carril':>6} {'z[m]':>5} {'alcanz.':>8} {'w_ins/w_max':>13} "
          f"{'w_tramo_min':>11} {'sen_q3':>7} {'sen_q5':>7} {'hombro[m]':>9}")
    for f in tipos:
        print(f"{f['tipo']:<9} {f['boca']:<10} {f['carril']:>+6d} {f['altura_boca_m']:>5.2f} "
              f"{f['puntos_alcanzables']:>8} {f['w_insercion_min']:>6.3f}-{f['w_insercion_max']:<6.3f} "
              f"{f['w_tramo_min']:>11.3f} {f['sen_q3_min']:>7.3f} {f['sen_q5_min']:>7.3f} {f['hombro_min_m']:>9.3f}")
    print("\nresultados/espacio_trabajo.csv, resultados/espacio_tipos.csv, resultados/espacio_trabajo.png")


if __name__ == "__main__":
    main()

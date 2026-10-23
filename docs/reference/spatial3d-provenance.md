# Spatial 3D physical reference provenance

W03 status: **source written; data entered pending V01 review**. The tables are
entered from publications, without running the simulator, fitting coefficients,
generating expected outputs, or acquiring a raw measurement payload. The index is
[`migration/spatial3d/references.json`](../../migration/spatial3d/references.json).

## Rigid bicycle benchmark

Source: J. P. Meijaard, J. M. Papadopoulos, A. Ruina and A. L. Schwab,
*Linearized dynamics equations for the balance and steer of a bicycle: a benchmark
and review*, Proceedings of the Royal Society A 463 (2007), 1955–1982,
[DOI 10.1098/rspa.2007.1857](https://doi.org/10.1098/rspa.2007.1857).
The [author-hosted published paper](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf)
is linked from [Schwab's benchmark page](https://arendschwab.com/research/benchmarkbicycle/).

The `meijaard2007.json` fixture preserves all 27 `WhippleParameters` fields.
Table 1, p. 1960 supplies 25 design parameters and gravity; speed is a variable.
The C++ default `v_mps=0` is explicitly identified as the Table 2(a) zero-speed
case, rather than represented as a fixed design parameter. Table 2, p. 1971 supplies
both complex weave roots and the real capsize/castering roots at 2 and 6 m/s,
plus the weave and capsize transition speeds. Each entry names its source location.

Coordinates remain those of the publication: forward x, right y, downward z.
The signs of `zB`, `zH` and the off-diagonal inertia components are retained.
Conversion into the simulation's world convention belongs to the benchmark model
builder. The comparison requires the paper's rigid bodies and ideal no-slip rolling
constraints; it does not validate a compliant tyre by itself.

The paper states Royal Society copyright, not a Creative Commons license. This
fixture contains attributed numerical facts only; no publication images, passages
or author software are included. V01 reviews transcription; V06 implements and
qualifies the independent matching benchmark.

## Lateral tyre cross-check

Source: G. Dell'Orto, G. Mastinu, R. Happee and J. K. Moore,
*Measurement of the lateral characteristics and identification of the Magic Formula
parameters of city and cargo bicycle tyres*, Vehicle System Dynamics 63(2),
308–332 (online 2024),
[DOI 10.1080/00423114.2024.2338143](https://doi.org/10.1080/00423114.2024.2338143).
The [TU Delft published PDF](https://pure.tudelft.nl/ws/portalfiles/portal/187282473/Measurement_of_the_lateral_characteristics_and_identification_of_the_Magic_Formula_parameters_of_city_and_cargo_bicycle_tyres.pdf)
declares [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
The changes here are selection, transcription and unit conversion.

`tyre-lateral.json` contains all six Schwalbe Big Ben Plus T01 entries from Table 2
(article p. 7; published p. 314). The tested tyre is 55-406, on the stated cargo rim,
at 400 kPa; loads are 411/526 N and camber is −5/0/+5 degrees. Table 1 supplies the
dimensions. Section 3 reports the 9.3 km/h rolling speed and 1.5% stiffness
uncertainty. No confidence level or coverage factor is invented.

Equation (3) defines stiffness at zero slip angle. The stored zero slip angle is
that derivative's evaluation point, not a fabricated force measurement. Conversion
is `N/rad = (N/deg) * 180/pi`; speed is converted using `km/h / 3.6`.
Original values and units are retained alongside SI values.

Every row is `cross_check_only`. These laboratory city/cargo tyre results are not
a calibrated coefficient profile for aggressive 2.8-inch low-pressure MTB tyres.
No raw files from the linked
[Zenodo dataset, DOI 10.5281/zenodo.7866646](https://zenodo.org/records/7866646)
were acquired. Its payload SHA-256 and payload license remain pending explicit
verification; the article's license is not silently assigned to the dataset.

## V01 handoff

`spatial.references` checks nonempty tables, physical scalar domains, SI
conversions, source locations and all C++ benchmark defaults against the entered
JSON at the prescribed `1e-12` tolerance. It does not fit the implementation to
expected values, read unacquired payloads, or claim a passed execution. V01 must
review these entries, acquire and identify any raw payload it uses, and run the
test on the qualified native build. Missing payload identity cannot qualify a
raw-dataset comparison.

# W34: независимая проверка Meijaard / Whipple

Статус: **source_written; numerical validation pending V06**. Численные
результаты не получены; код тестов и протокол подготовлены без сборки и запуска.

Первичный источник: Meijaard, Papadopoulos, Ruina, Schwab (2007),
[Linearized dynamics equations for the balance and steer of a bicycle](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf),
[DOI 10.1098/rspa.2007.1857](https://doi.org/10.1098/rspa.2007.1857).
Параметры — Table 1; независимые линейные коэффициенты — Appendix A A1–A27;
числовые матрицы — equations 6.1–6.4; контрольные корни — Table 2.

## Два независимых пути

Production builder создаёт четыре тела B/H/R/F с опубликованными массами,
центрами и тензорами инерции. Переход к z-up меняет z и Ixz. Ступицы лежат
на высотах rR/rF, рулевая ось пересекает плоскость на c впереди переднего
контакта. В этом режиме нет подвески, pneumatic-шины, drivetrain, grip,
демпфирования или встроенного контакта MuJoCo. Размер визуальной толщины
колеса не входит в уравнения идеального контакта.

Test-only constraint владелец вычисляет шесть rolling equations по Jacobian
материальной точки контакта колеса: normal, longitudinal, lateral для каждого
колеса. `mj_jac` уже содержит вклад spin; повторное добавление omega×r
запрещено. Производная A-dot-v вычисляется центральной разностью на manifold
с h=1e-7. Ускорения и реакции получаются из полного KKT блока, без повышения
жёсткости, псевдообратной матрицы или epsilon-регуляризации масс.

Независимый reference модуль вычисляет Appendix A непосредственно из
параметров. Он не читает model, qM или native Jacobian. Квартет корней
вычисляется из determinant polynomial в long double. Native reduced Jacobian
получается perturbation по roll, steer и их скоростям с проекцией на допустимые
координаты и скорости. Сравнение корней выполняется по лучшей перестановке,
чтобы порядок вывода eigensolver не создавал ложное различие.

## Acceptance V06

| Проверка | Допуск | Результат |
|---|---|---|
| Все параметры против независимого JSON | 1e-12 | pending |
| Reference матрицы и Table 2 roots | 1e-12 / 1e-9 | pending |
| Native roots при 2, 4, 6, 8, 10 m/s | max(3%, .05 / s) | pending |
| Корень с модулем <.05 / s | 5e-4 / s | pending |
| Изменение near-zero root при h refinement | 1e-4 / s | pending |
| Weave boundary | 4.29238253634111 ± .2 m/s | pending |
| Capsize boundary | 6.02426201538837 ± .2 m/s | pending |
| KKT residual | <=1e-10, полный rank | pending |
| Нелинейный rollout .5 s, weave | рост при 2, затухание при 6 m/s | pending |

Границы локализуются бисекцией до .01 m/s с обязательной проверкой смены
знака. Перед этим контролируется сходимость tangent differences h, h/2, h/4
и Richardson extrapolation. Результаты печатаются тестом; файл не содержит
фиктивных измерений и не квалифицирует основную MTB-модель.

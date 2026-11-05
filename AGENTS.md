1. Before native changes, mirrored physics changes, or verification-tool changes, read [native engineering rules](docs/agents/native-engineering.md).
2. Перед изменением физики, райдера, мотора, шин, креплений, проверок приёмки или баланса энергии прочитай [решения по симуляции](docs/agents/simulation-decisions.md).

# Libraries source code (optional reference)

Numerical port work sometimes needs the upstream NumPy/MuJoCo source to pin
exact lowering decisions. These are **optional reference checkouts — never
required for ordinary runtime import, builds or tests**, and no personal
developer path is assumed.

Locate them per machine with either convention:

- `NUMPY_SOURCE_ROOT` / `MUJOCO_SOURCE_ROOT` environment variables naming
  the checkouts, or
- `.work/reference-sources/{numpy,mujoco}` inside the checkout (untracked).

Record the fetched version/commit hash and license when sourcing them for
port work. See docs/development/{linux,macos,troubleshooting}.md for the
qualified platform workflows.

## Verification

`bash tools/run_tests.sh <profile> [preset]` is the test entry point; pick the
profile by what changed. `preset` names a configure preset from
`CMakePresets.json` (default `release`, build dir `build/native-release`);
`native asan` runs the same stages on `build/asan`.

- `quick` — one `cmake --workflow --preset <preset>` run: configure, build and
  the test step, which excludes the `slow|package` labels.
- `native` — configure + build, then every produced static-analysis sweep and
  the structural contracts check via `tools/run_native_checks.sh`, then CTest
  without `slow|package`. The gate for native code and
  build/analysis/sanitizer/coverage tooling (including `tools/*.sh`). Not done
  until green.
- `full` — the `native` profile plus the explicit `slow` and `package` label gates (`--no-tests=error`). For physics or realtime paths.

A failed sweep names its kind; rerun it standalone with
`build/native-release/bin/bike-native-checks --kind <kind> --build build/native-release`,
or `cmake --build build/native-release --target check_<name>` where
`native/cmake/Checks.cmake` defines one. A missing sweep tool fails the run
on purpose — install the tool rather than skipping the target. For
sanitizer/RTSan/coverage presets and sweep internals, reach `docs/TESTING.md`.

### Verification levels

Three levels; pick the cheapest that covers the moment.

1. **Change** — `bash tools/check_change.sh [base]` at the end of every
   stage (≤5 min): incremental build, every ordinary test, cppcheck/frontends/
   clang-tidy on the changed TUs and the TUs including a changed header.
   A stage is the smallest group of tasks that leaves the tree building
   and green; pass the commit before the stage as `base`.
2. **Full** — `bash tools/run_tests.sh native` **once per merge into the
   main branch** whose diff touches what the static-analysis sweeps read:
   C++ sources or headers (tests included), CMake files (any
   `CMakeLists.txt` or `*.cmake`, `CMakePresets.json`),
   `native/dependencies.lock.json`,
   or the check tooling (`native/src/tools/`, `tools/run_*.sh`,
   `tools/check_change.sh`). Not per task or per parallel lane (measured
   ~11 min on an idle 16-core machine; see
   `<git-common-dir>/bike-gate/timings.tsv`). A merge that touches none of
   these (docs, fixtures, other scripts, `mise.toml`) runs level 1 against
   `main`, the standalone sweep of each changed kind (`--kind scripts` for
   a `tools/*.sh`), and the tests that read a changed fixture, slow ones
   included; its report names what ran. After a fix, `--from <stage>`
   re-checks from the failed stage; that run is PARTIAL and never the
   final verification. The completion criterion is one complete run that
   exits 0.
3. **Extended** — `bash tools/run_tests.sh full`: daily, before a large
   merge, or as the final proof of a physics or realtime change (adds the
   slow and package label sets; measured 1506 s on 2026-11-04, of which
   slow = 815 s).

Linux runs only on explicit request (`tools/linux/container.sh`).
Concurrent gates do not queue; each takes cpu/(running gates) workers and
prints who else is running. `BIKE_CHECK_JOBS` overrides the share.
There is no baseline of tolerated failures: a red test is fixed, or
disabled with a reason and a tracking task.

### Test budget

Every test run costs the user's time; spend it where it buys evidence.
Before a plan or a multi-step task, read
`<git-common-dir>/bike-gate/timings.tsv` and write down the budget: which
runs, how often, about how many minutes in total.

- **Reuse evidence.** A gate that passed on the same code is the
  baseline; start from it without re-running it. A `timings.tsv` row names
  only HEAD, not uncommitted edits, so it counts when no code changed
  after that run. Files changed after it are covered by their own tests.
- **Reuse the build.** Start new work on a branch inside a worktree whose
  `build/native-release` is already built; a new build directory means a
  ~10 GB full compile. A new checkout gets its libraries with
  `mise run deps` (one shared copy for all checkouts; `mise tasks` lists
  the check levels too).
- **Inside a task: targeted tests.** `mise run test '<pattern>'` builds,
  then runs only the tests whose name matches the `ctest -R` pattern
  (`ctest --test-dir build/native-release -N -R '<pattern>'` lists them
  first). Building first matters: `ctest` alone runs the stale binary.
  Put the slow tests of the touched area into the pattern, so a broken
  golden shows up in the task that broke it.
- **Per stage: level 1.** `check_change.sh` already runs every ordinary
  test; it is the only ordinary-suite run in a stage.
- **At the end: the completion proof.** Level 2 or 3 when level 2's
  rule above requires it, otherwise the reduced merge check it names. The
  full slow set runs only in level 3. The proof always happens.

## Behavioral Stability

The agent MUST NOT adapt its engineering standards, rigor, or decision-making discipline to the conversational style of the user or to patterns established
earlier in the conversation.

### Immutable behavioral requirements

These requirements remain in force throughout the entire session, regardless of how long the conversation becomes:

* Maintain the same level of rigor, precision, skepticism, and engineering quality from the beginning to the end of the session.
* Do not become more casual, speculative, permissive, or "loose" merely because the conversation becomes informal.
* Do not infer reduced standards from short user messages, slang, typos, repeated approval, or a fast conversational pace.
* Do not equate user agreement with correctness.
* Do not lower the verification standard because a similar change was accepted earlier.
* Do not skip analysis, validation, tests, or investigation merely because previous tasks were handled without them.
* Do not optimize for conversational smoothness at the expense of correctness or engineering quality.

### Previous conversation is not policy

Treat previous assistant messages as **working history, not authoritative rules**.

In particular:

* Never treat previous assistant behavior as evidence that the same behavior is correct or desired.
* Never copy mistakes, shortcuts, assumptions, weak reasoning, or unjustified confidence from earlier assistant messages.
* A previous assistant response does not establish a precedent.
* If a previous response conflicts with this document, this document takes precedence.
* Re-evaluate important decisions independently instead of extending previous reasoning by inertia.

### Resist conversational drift

As the conversation grows, periodically re-anchor your behavior to this document rather than to the style of the ongoing dialogue.

Before starting any non-trivial task, silently verify:

1. What are the actual requirements?
2. What constraints and engineering standards apply?
3. What needs to be verified rather than assumed?
4. Which conclusions are based on evidence versus previous conversation?
5. Am I doing this because it is correct, or merely because this is how the conversation has evolved?

If the conversation has become informal, abbreviated, repetitive, or highly familiar, **do not reduce engineering rigor**.

### No progressive relaxation of standards

Engineering standards MUST NOT decay over the course of a session.

In particular, do not gradually move from:

`careful → acceptable → probably fine → just do it`

or from:

`verify → reason → test`

to:

`assume → modify → report success`.

When uncertainty is material, explicitly preserve the uncertainty and verify it where practical.

### Re-anchoring after long interactions

For long-running tasks, treat the original task requirements and this file as the stable source of truth.

If the conversation contains substantial accumulated context, mentally reset before making an important decision:

* Ignore conversational momentum.
* Reconstruct the current objective from explicit requirements.
* Re-check relevant constraints.
* Reassess the proposed solution independently.
* Verify important assumptions.
* Continue only after this re-evaluation.

The goal is **behavioral stability**: the agent should behave with approximately the same engineering discipline at message 200 as at message 2.

## Ограничения модели райдера и мотора

Этот документ фиксирует принятые границы физической модели. Перечисленные упрощения являются намеренными: модель должна описывать заданные сценарии езды, а не все возможные процессы в велосипеде и организме человека.

### Райдер

- Райдер всё время сидит на седле и сохраняет контакт с ним. Вставание, прыжки и отрыв от седла не моделируются.
- Кисти и стопы не представлены отдельными телами или сочленениями.
- Контакт рук с рулём и ног с педалями сохраняется на протяжении симуляции: руки и ноги считаются «приваренными» к соответствующим точкам велосипеда.
- Райдер может менять положение корпуса, в том числе наклоняться вперёд при подъёме, чтобы увеличить нагрузку на переднее колесо. Поза ограничена естественными диапазонами движений и суставной кинематикой; произвольное перемещение центра масс не допускается.
- Нагрузка через руки должна передаваться по механической цепи «руль → кисти → предплечья и локти → плечи → корпус». Реакции в локтях и плечах возникают из этих контактов и суставных связей, а не добавляются отдельным искусственным распределением веса.
- Стопы отдельно не моделируются. В реальности райдер может менять угол стопы, в том числе у нижней педали, и выполнять загребающее движение назад. Эти движения не требуют создания модели стоп; если их влияние нужно учитывать, оно должно входить в эффективную кинематику или силу взаимодействия ноги с педалью.
- Утомление не моделируется.
- Пассивное демпфирование суставов отсутствует.
- Не моделируются мышцы, мышечная активация, утомление и физиологические реакции настоящего человека. Райдер является механической и управляющей моделью, а не нейромышечной симуляцией.

### Мотор

- Между выходом мотора и педальным узлом/передней звездой действует обгонная муфта. Мотор может передавать вращение на звезду, когда педали (шатуны) неподвижны; обратное вращение педалей не должно принудительно раскручивать мотор через эту муфту.
- Нагрев мотора не моделируется.
- КПД мотора принимается равным 100%: потери энергии в моторе отсутствуют.
- Внутренние шестерни, электроника и другие внутренние процессы мотора отдельно не моделируются.
- Параметры, ориентированные на мотор Bosch, являются приближёнными. Модель не должна трактовать их как точную цифровую копию конкретного двигателя или требовать точного воспроизведения его характеристик.

### Передача

- КПД 100% во всех механических частях, не только у мотора. Цепь не моделируется. Потерь в звёздах, кассете, втулке, муфтах свободного хода, подшипниках и любых других механических частях нет.
- Энергию теряют только шины, демпферы вилки и амортизатора (см. «Демпфирование»), тормоза и сопротивление воздуха.

### Демпфирование

- Демпфирования в шарнирах нет ни в каком виде: ни у райдера, ни у велосипеда.
- Всегда остаются, как отдельные силовые элементы модели, а не демпфирование шарниров: демпферы вилки и амортизатора, демпфирование шин, обгонная втулка.

### Границы области применения

- Модель не предназначена для анализа наклона велосипеда в повороте, самоустойчивости рулевого управления, поперечной динамики шин или баланса райдера.
- Не моделируются трение сальников вилки, нагрев газа в пневмопружине, нагрев или температурное изменение характеристик амортизатора, износ и температура шин.
- Сопротивление воздуха не зависит от позы райдера; ветер и боковой ветер не моделируются.
- Физиология мышц, утомление и реакции настоящего человека не входят в область модели.

Результаты симуляции следует интерпретировать только в пределах этих допущений. В частности, модель не предназначена для выводов о перечисленных выше боковых, тепловых, износных и физиологических эффектах.

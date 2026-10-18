# rt_cell

Детерминированный планировщик реального времени и парсер бинарного промышленного
протокола на **C++20** без динамических аллокаций.

**Zero-allocation** · **Lock-free SPSC** · **CRC-16-CCITT** · **Jerk-limited S-curve** · **AABB/OBB collision intervals** · **ASan/UBSan/TSan clean**

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue)
![deps](https://img.shields.io/badge/deps-stdlib%20only-informational)
![allocations](https://img.shields.io/badge/new%2Fmalloc%20after%20init-0-brightgreen)
![sanitizers](https://img.shields.io/badge/ASan%2BUBSan%2BTSan-pass-green)
![MIT](https://img.shields.io/badge/license-MIT-yellow)

Проект **полностью создан локальной моделью** (Strata, квантованный Qwen, IQ3)
на рабочем станке RTX 3090 + Xeon E5-2699 v3 без доступа в интернет — как ответ
на тезис «локальные модели не тянут сложный системный код».
Как это работало: [docs/LOCAL-MODEL.md](docs/LOCAL-MODEL.md).

---

## Мотивация

Типовой спор двух миров: «разработчики не шарят за производство» и «локальная
модель не напишет низкий уровень». Демонстрируется пересечение — реальный
класс задач цеха:

- закрытый контур ГПЯ: 3 робота-манипулятора с общими зонами досягаемости, 2 станка ЧПУ;
- облачные API физически недоступны → весь цикл (код, сборка, верификация) локальный;
- требования промышленного уровня: детерминизм, отсутствие аллокаций в горячем
  пути, корректные барьеры памяти, валидация каждого байта линии связи.

## Быстрый старт

Требуется GCC ≥ 13 (полный C++20). Других зависимостей нет.

```bash
make run        # test_plain + ASan/UBSan + TSan, один проход
```

Ожидаемый финал каждой сборки: `=== RESULT: PASS (0 failures) ===`.
Сборки: `test_plain` (100 000 кадров, детектор аллокаций включён),
`test_asan` (ASan+UBSan), `test_tsan` (TSan; на ядрах 6.8+ с новым раскладом
виртуальной памяти запуск через `setarch $(uname -m) -R` — уже в `make run`).

## Верификация

| Критерий | Доказательство | Вердикт |
|---|---|---|
| Сборка без предупреждений | `-Wall -Wextra -Wpedantic -Wshadow -Werror`, g++ 13.3 | PASS |
| CRC-16-CCITT-FALSE | эталон `«123456789» → 0x29B1` в `static_assert` + сверка с `binascii.crc_hqx` | PASS |
| Битый кадр не попадает в планировщик | стресс 100k: `crc=50000` отброшено, `planned+rejected = ok = 50000` | PASS |
| Ноль аллокаций в горячем пути | подменённый глобальный `operator new` → `bad_alloc`; 100k кадров, ни одного срабатывания | PASS |
| Конфликт траекторий в зоне | AABB и OBB: интервалы occupancy разъехались, робот с высшим приоритетом побайтно нетронут, оба доехали до цели | PASS |
| Цикл не останавливается | скорость victim в середине окна зоны > 0.1 м/с после `Respeed` | PASS |
| Отсутствие гонок и дедлоков | TSan: стресс 20k кадров, 2 потока, exit 0 | PASS |

Подробный протокол прогонов и разбор найденных при отладке дефектов:
[docs/VERIFICATION.md](docs/VERIFICATION.md).

## Архитектура

```mermaid
flowchart LR
    NET[Поток TCP/UDP] --> RING["ByteRing 256 B\n(std::array, alignas)"]
    RING --> PARSER["FrameParser\nmagic → len → CRC-16"]
    PARSER -- "CrcError / мусор" --> DROP[Ресинк побайтно]
    PARSER -- "FrameMsg, alignas(64)" --> SPSC["SPSC ring\nrelease/acquire, lock-free"]
    SPSC --> CONS[Поток планировщика]
    CONS --> ADMIT["admit_traj\nпуть tcp → target"]
    ADMIT --> OCC["Интервалы occupancy\nslab-тест отрезка × AABB/OBB"]
    OCC --> CONF{Пересечение\nво времени?}
    CONF -- да --> MIT["Respeed ×0.85⁶ → DelayStart\nS-curve профиль, без остановки"]
    MIT --> ADMIT
    CONF -- нет --> RUN[Исполнение]
```

Ключевые инварианты:

1. После инициализации — ноль `new`/`malloc`; все структуры статические
   (`std::array`, `alignas(64)`), парсер работает через `std::span` побайтно
   (нет UB strict-aliasing).
2. Планировщик однопоточный; единственный канал «сеть → цех» — SPSC-очередь
   с монотонными `uint64`-индексами и `memory_order_release/acquire`.
3. Разрешение коллизий детерминировано: фиксированный порядок обхода пар,
   максимум 8 проходов, максимум 6 попыток `Respeed`.

## Протокол в двух строках

Big-endian, заголовок 18 байт, максимум 84:

```text
magic(2) ver(1) type(1) len(2) seq(2) ts_ms(4) axis_mask(4) axes(2)
  | coords axes×4 (Q16.16) | crc16(2)
```

Тестовый вектор (команда TRAJ_CMD, seq=0x1234, цель (10, 3, 0)) и полная
таблица полей: [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Производительность

Xeon E5-2699 v3, `-O2`: полный прогон тестового цикла (парсинг + CRC + SPSC +
планировщик, 100k кадров в 2 потока) — **≈ 40 мс**, то есть
**≈ 2.5 млн кадров/с** на одном ядре 2015 года. Для шины цеха на сотни кадров
в секунду запас — три порядка.

## Документация

| Документ | Содержание |
|---|---|
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | формат кадра, CRC-параметры, Q16.16, автомат парсера, тест-векторы |
| [docs/TRAJECTORY.md](docs/TRAJECTORY.md) | 7-сегментный jerk-limited профиль: три аналитические ветки, доказательства инвариантов |
| [docs/SCHEDULING.md](docs/SCHEDULING.md) | Job-Shop, интервалы occupancy, Respeed/DelayStart, детерминизм |
| [docs/CONCURRENCY.md](docs/CONCURRENCY.md) | lock-free SPSC: барьеры памяти, почему без CAS, padding |
| [docs/VERIFICATION.md](docs/VERIFICATION.md) | команды, вывод санитайзеров, матрица приёмки, кейсы дефектов |
| [docs/LOCAL-MODEL.md](docs/LOCAL-MODEL.md) | как это написано локальной моделью: стенд, цикл генерация→верификация, уроки |

## Структура

```text
rt_cell/
├── Makefile
├── include/rt/
│   ├── protocol.hpp    # кадр, CRC-16 (constexpr-таблица), ByteRing, FrameParser, FrameWriter
│   ├── spsc.hpp        # lock-free SPSC ring buffer
│   ├── geometry.hpp    # Vec3, AABB, OBB, slab-пересечение отрезка с боксом
│   ├── traj.hpp        # S-curve jerk-limited профиль + инверсия времени
│   └── scheduler.hpp   # роботы, зоны, станки, occupancy, mitigation, Job-Shop
└── tests/main.cpp      # 6 модульных тестов + стресс + zero-alloc guard
```

## Ограничения (честно)

- Робот — материальная точка TCP по полилинии; кинематика манипулятора (ориентация
  инструмента, самостолкновение) вне объёма.
- Планировщик однопоточный и итеративный (≤ 8 проходов): достаточно для трёх
  роботов, не доказан для произвольных конфигураций.
- `DelayStart` — фазовый сдвиг, а не перестройка всего расписания.
- Станки учтены как Job-Shop ресурс «занят до времени T», без оснастки/инструмента.

## Лицензия

MIT — см. [LICENSE](LICENSE).

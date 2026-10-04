"""Shared console output for the physical ride viewer.

One ``rich.Console`` owns styling for every line the physical viewer emits, so
the HUD, the column legend and the service messages agree on when ANSI is
written (isatty, ``NO_COLOR``, ``TERM=dumb``) and on never cropping output:
``soft_wrap`` lets a long line wrap at the terminal edge like plain print
would, instead of truncating the channels at the tail.
"""
from rich.console import Console
from rich.text import Text

console = Console(soft_wrap=True)

# Physical viewer key map, shared by the session help and the viewer banner.
PHYSICAL_KEY_MAP = ("Space brakes; ,/. brake strength; R reset; "
                    "C/1/2 camera; T telemetry; G markers.")
PHYSICAL_HELP_NOTE = ("W/S changes the target only in ideal_speed_control. "
                      "Material and drive tuning is fixed per run.")


def styled(columns):
    """Assembles styled segments: ``[(text, style), ...] -> Text``.

    An empty style string means "unstyled", same as None.
    """
    return Text.assemble(*[(text, style or None) for text, style in columns])


def info(text):
    """Service chatter: '[bike-ride] ...' lines. Dim, below the data stream."""
    console.print(text, style="dim")


def event(text):
    """Constraint/monitor failure report. Loud by definition."""
    console.print(text, style="bold red")


def outcome(text):
    """Run-termination report ('[RUN ENDED] ...')."""
    console.print(text, style="bold yellow")


def key_help(text):
    """One help line: dim prose, the ';'-separated key tokens bold."""
    line = Text(style="dim")
    for index, clause in enumerate(text.split(";")):
        clause = clause.strip()
        if not clause:
            continue
        if index:
            line.append("; ", style="dim")
        key, _, rest = clause.partition(" ")
        line.append(key, style="bold")
        if rest:
            line.append(" " + rest, style="dim")
    console.print(line)


def print_help():
    """The physical viewer's key map: dim prose, bold key tokens."""
    info("Physical ride:")
    key_help(PHYSICAL_KEY_MAP)
    info(PHYSICAL_HELP_NOTE)


__all__ = ["console", "styled", "info", "event", "outcome", "key_help",
           "print_help", "PHYSICAL_KEY_MAP", "PHYSICAL_HELP_NOTE"]

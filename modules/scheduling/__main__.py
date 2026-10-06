"""List every registered dispatch rule, planner, speed law and setpoint gate."""

from modules.scheduling.registry import describe

print(describe())

"""Reference solutions: each task done through the hub's API alone, as a model would have to.

A solution gets one thing, an `Api` (solutions/api.py): GET and POST on the hub's REST API, and pictures.
It never sees the scene file, the simulation or the referee. What a model would do with its eyes (find the
red thing in a picture) a solution does with a few lines of colour thresholding (solutions/eyes.py); what
a model would do with its judgement (which of these things is the shelf) a solution does with a rule
written for that task. Everything about where things are comes from the hub.

    python sim/scripts/solve.py                    # every task that has a solution, scored by the referee
    python sim/scripts/solve.py shelf on_the_shelf

SOLVERS maps (scene, task) to its solution. A task missing from it has none yet; solve.py says why.
"""
from .tasks import NO_SOLUTION, SOLVERS  # noqa: F401

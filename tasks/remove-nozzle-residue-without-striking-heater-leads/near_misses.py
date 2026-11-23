"""Near misses and shortcuts. Pose-only cases fail closed: a score needs the in-run history (bristle contact, strike,
damage, hands-clear and stillness records), so no pose by itself can score. The removal rule, its thresholds, the
strike and damage latches and every tier boundary are tested on the rule itself in validation/check_model.py, which
this file runs (the stock checker imports this file)."""
import pathlib, runpy
TOP = .7475
CASES = [
    ('no op', {}, 0., False),
    ('brush held under the nozzle, never released', {'brass_brush': ((.175, 0., TOP + .159), None, None)}, 0., False),
    ('brush dropped on the build plate', {'brass_brush': ((.20, .05, TOP + .106), None, None)}, 0., False),
    ('brush put back in its place without brushing', {'brass_brush': ((.250, .265, TOP + .006), None, None)}, 0., False),
    ('catch tray knocked off the build plate', {'service_tray': ((.10, -.20, TOP), None, None)}, 0., False),
    ('catch tray pushed out from under the nozzle', {'service_tray': ((.20, -.08, TOP + .100), None, None)}, 0., False),
]
runpy.run_path(str(pathlib.Path(__file__).parent / 'validation' / 'check_model.py'), run_name='__contract_check__')

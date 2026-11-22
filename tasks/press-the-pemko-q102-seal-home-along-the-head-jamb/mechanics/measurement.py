"""Read physical evidence, without substituting a simulator result for a measurement."""
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent


def calibration_errors(record=None):
    r = json.loads((HERE / 'calibration.json').read_text()) if record is None else record
    errors = []
    if r.get('status') != 'MEASURED':
        errors.append('Physical insertion-force measurement is missing.')
    if r.get('product') != 'Pemko Q102BL':
        errors.append('Calibration product must be Pemko Q102BL.')
    for key, value in [('kerf_width_mm', 3.175), ('kerf_depth_mm', 9.525),
                       ('pressed_length_mm', 812.8 / 34)]:
        try:
            number = float(r[key])
            if not math.isfinite(number) or abs(number - value) > .025:
                errors.append(f'{key} does not match the specified specimen.')
        except (KeyError, TypeError, ValueError):
            errors.append(f'{key} is missing.')
    readings = r.get('force_readings_N', [])
    if (not isinstance(readings, list) or len(readings) < 3 or
            any(not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0 for x in readings)):
        errors.append('At least three positive physical force readings are required.')
    for key in ['method', 'instrument', 'measured_at_utc']:
        if not isinstance(r.get(key), str) or not r[key].strip():
            errors.append(f'{key} is missing.')
    for key in ['instrument_resolution_N', 'model_contact_stiffness_N_per_m', 'model_contact_damping_Ns_per_m']:
        v = r.get(key)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            errors.append(f'{key} is missing.')
    evidence = r.get('evidence_file')
    if not isinstance(evidence, str) or not evidence:
        errors.append('Physical evidence file is missing.')
    else:
        file = (HERE / evidence).resolve()
        if HERE not in file.parents or not file.is_file():
            errors.append('Evidence must be a file inside the task folder.')
        elif hashlib.sha256(file.read_bytes()).hexdigest() != r.get('evidence_sha256'):
            errors.append('Evidence hash does not match.')
    return errors


def readiness_errors(record=None):
    r = json.loads((HERE / 'calibration.json').read_text()) if record is None else record
    errors = calibration_errors(r)
    native = r.get('native_insertion_force_readings_N', [])
    if not isinstance(native,list) or len(native)<3 or any(not isinstance(x,(int,float)) or not math.isfinite(x) or x<=0 for x in native):
        errors.append('Native contact-resistance comparison is missing.')
    elif r.get('force_readings_N') and not errors and min(native)<max(r['force_readings_N']):
        errors.append('Native resistance is below the physical calibration envelope.')
    # These aggregate checks cannot verify a force-versus-travel curve or a
    # video: an independent reviewer must also inspect the referenced traces.
    return errors


def require_calibration():
    record=json.loads((HERE/'calibration.json').read_text())
    if record.get('status')=='ESTIMATED':
        errors=[]
        if not record.get('estimate_basis') or not record.get('estimate_source'):
            errors.append('Estimate basis/source missing.')
        for key in ['estimated_insertion_force_N','model_contact_stiffness_N_per_m','model_contact_damping_Ns_per_m']:
            v=record.get(key)
            if not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:errors.append(key+' missing.')
    else:
        errors = calibration_errors(record)
    if errors:
        raise RuntimeError('NEEDS-MORE: ' + ' '.join(errors))
    return record


if __name__ == '__main__':
    errors = readiness_errors()
    print('\n'.join(errors) if errors else 'Calibration records complete; inspect force curves and final native evidence before readiness.')
    raise SystemExit(bool(errors))

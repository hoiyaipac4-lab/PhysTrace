# Task and Record Protocol

1. Record engine, runtime, task, asset version, coordinates and units.
2. Specify geometry, mass, inertia, materials, initial state, input, timestep and termination rules.
3. Read back effective contact and solver parameters from the engine.
4. Align input, realized state, motion and contact records by time.
5. Apply the task score and report attempted, evaluated and failed trial counts.

The cradle example uses metres, kilograms, seconds, radians, newtons and newton-metres. Its input is passive release under gravity. The example record identifies available contact channels.

First-peak screening uses an absolute relative error threshold of 0.10. Energy screening uses a maximum potential-to-initial-energy ratio of 1.05. The supplied analysis scans six multipliers from 0.5 to 2.0 for these allowances.

```sh
python reproduction/protocol/validate_record.py
```

The checker validates fields, units, clock relationships, passive input and digest syntax. Pass a JSON file as an argument to check another cradle record.

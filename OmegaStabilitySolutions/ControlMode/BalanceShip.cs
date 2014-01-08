using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS.ControlMode {
    class BalanceShip : ControlMode {
        private EngineManager engineManager;

        private float prevDampScalar;

        public override Vessel vessel { get; set; }
        
        public override void onFixedUpdate() {
            if (engineManager == null) return;
            engineManager.update();

            var numEngines = engineManager.OperationalEngines.Count();

            if (numEngines > 0) {
                float dampAmount = 0;
                float torque = 0;
                foreach (var engine in engineManager.OperationalEngines) {
                    if (engine.useEngineResponseTime) {
                        dampAmount += ((engine.engineAccelerationSpeed + engine.engineDecelerationSpeed) / 2) * engine.maxThrustMagnitude;
                    } else {
                        dampAmount += engine.maxThrustMagnitude;
                    }
                    torque += engine.maxThrustMagnitude;
                }
                dampAmount /= torque;
                vessel.vesselSAS.dampingModeScalar = Mathf.Clamp(prevDampScalar * dampAmount, 0.25f, 0.95f);
            }

            bool success = false;

            Vector3 currentTorque = calculateCurrentTorque();

            var thrustSolver = new LP.MaxSolver();
            var thrustProblem = thrustSolver.Problem;

            var torqueSolver = new LP.MaxSolver();
            var torqueProblem = torqueSolver.Problem;

            var fuelSolver = new LP.MaxSolver();
            var fuelProblem = fuelSolver.Problem;

            Vector3 wheelTorque = Vector3.zero;
            foreach (var wheel in vessel.GetModules<WheelController>().Where(w => w.Operational)) {
                wheelTorque += wheel.torqueStrength;
            }

            // Create variables
            EngineVariableSet[] engineVariables = engineManager.OperationalEngines.Select(e => new EngineVariableSet(e)).ToArray();
            LP.Variable wheelX = new LP.BoundedVariable(-wheelTorque.x, wheelTorque.x);
            wheelX.Name = "wheelX";
            LP.Variable wheelY = new LP.BoundedVariable(-wheelTorque.y, wheelTorque.y);
            wheelY.Name = "wheelY";
            LP.Variable wheelZ = new LP.BoundedVariable(-wheelTorque.z, wheelTorque.z);
            wheelZ.Name = "wheelZ";

            LP.Variable curThrottle = new LP.BoundedVariable(1, 1);
            curThrottle.Name = "curThrottle";
            LP.Variable absThrottle = new LP.BoundedVariable();
            absThrottle.Name = "absThrottle";

            LP.Variable[] allVariables =
                engineVariables.SelectMany(ev => ev.all())
                .Concat(new []{wheelX, wheelY, wheelZ, curThrottle, absThrottle})
                .ToArray();

            thrustSolver.Variables = allVariables;
            torqueSolver.Variables = allVariables;
            fuelSolver.Variables = allVariables;

            // Create Constraints
            var thrustMoment = new LP.TriConstraint();
            thrustProblem.addConstraint(thrustMoment);

            var torqueMoment = new LP.BiConstraint();
            var torqueThrust = new LP.Constraint(thrustProblem.Objective, 0);
            torqueProblem.addConstraint(torqueMoment);
            torqueProblem.addConstraint(torqueThrust);

            var fuelMoment = LP.TriConstraint.Clone(thrustMoment);
            var fuelThrust = torqueThrust;
            fuelProblem.addConstraint(fuelMoment);
            fuelProblem.addConstraint(fuelThrust);

            if (wheelTorque.sqrMagnitude > 1e-5) {
                Vector3d xcomp = vessel.transform.InverseTransformDirection(vessel.ReferenceTransform.rotation * new Vector3d(-1.0, 0.0, 0.0));
                Vector3d ycomp = vessel.transform.InverseTransformDirection(vessel.ReferenceTransform.rotation * new Vector3d(0.0, -1.0, 0.0));
                Vector3d zcomp = vessel.transform.InverseTransformDirection(vessel.ReferenceTransform.rotation * new Vector3d(0.0, 0.0, -1.0));

                thrustMoment[wheelX] = xcomp;
                thrustMoment[wheelY] = ycomp;
                thrustMoment[wheelZ] = zcomp;
            }

            var engineConstraints = new EngineConstraintSet[numEngines];
            
            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                var ev = engineVariables[i];
                var c = engineConstraints[i] = new EngineConstraintSet();

                thrustProblem.addConstraint(c);
                torqueProblem.addConstraint(c);
                fuelProblem.addConstraint(c);

                Vector3d origin = Vector3d.zero;
                Vector3d apo = Vector3d.back;

                double range = engine.gimbalRange * (Math.PI / 180.0);
                double s_t = Math.Sin(range);
                double c_t = Math.Cos(range);

                var lowerPX = new Vector3d(c_t, 0, s_t);
                var lowerPY = new Vector3d(0, c_t, s_t);
                var lowerNX = new Vector3d(-c_t, 0, s_t);
                var lowerNY = new Vector3d(0, -c_t, s_t);

                {
                    Vector3d lowerPXNorm = lowerPX.normalized;
                    Vector3d lowerPYNorm = lowerPY.normalized;
                    Vector3d lowerNXNorm = lowerNX.normalized;
                    Vector3d lowerNYNorm = lowerNY.normalized;

                    c.lowerPositive[ev.NX] = -(c.lowerPositive[ev.PX] = new Vector2d(lowerPXNorm.x, lowerPYNorm.x));
                    c.lowerPositive[ev.NY] = -(c.lowerPositive[ev.PY] = new Vector2d(lowerPXNorm.y, lowerPYNorm.y));
                    c.lowerPositive[ev.NZ] = new Vector2d(-lowerPXNorm.z, -lowerPYNorm.z);
                    c.lowerPositive[ev.lowerPositiveX] = new Vector2d(1, 0);
                    c.lowerPositive[ev.lowerPositiveY] = new Vector2d(0, 1);
                    c.lowerPositive.RHS = new Vector2d(0, 0);

                    c.lowerNegative[ev.NX] = -(c.lowerNegative[ev.PX] = new Vector2d(lowerNXNorm.x, lowerNYNorm.x));
                    c.lowerNegative[ev.NY] = -(c.lowerNegative[ev.PY] = new Vector2d(lowerNXNorm.y, lowerNYNorm.y));
                    c.lowerNegative[ev.NZ] = new Vector2d(-lowerNXNorm.z, -lowerNYNorm.z);
                    c.lowerNegative[ev.lowerNegativeX] = new Vector2d(1, 0);
                    c.lowerNegative[ev.lowerNegativeY] = new Vector2d(0, 1);
                    c.lowerNegative.RHS = new Vector2d(0, 0);
                }

                Vector3d upxn;
                {
                    Vector3d pyc = Vector3d.Cross(lowerPY, lowerPX);
                    pyc.z += 1;
                    Vector3d nyc = Vector3d.Cross(lowerPX, lowerNY);
                    nyc.z += 1;
                    upxn = Vector3d.Cross(pyc, nyc);
                    upxn.Normalize();
                }

                var upperPX = new Vector3d(upxn.x, 0, upxn.z);
                var upperPY = new Vector3d(0, upxn.x, upxn.z);
                var upperNX = new Vector3d(-upxn.x, 0, upxn.z);
                var upperNY = new Vector3d(0, -upxn.x, upxn.z);
                double uRHS = -upxn.z;

                {
                    Vector3d upperPXNorm = upperPX.normalized;
                    Vector3d upperPYNorm = upperPY.normalized;
                    Vector3d upperNXNorm = upperNX.normalized;
                    Vector3d upperNYNorm = upperNY.normalized;

                    c.upperPositive[ev.NX] = -(c.upperPositive[ev.PX] = new Vector2d(upperPXNorm.x, upperPYNorm.x));
                    c.upperPositive[ev.NY] = -(c.upperPositive[ev.PY] = new Vector2d(upperPXNorm.y, upperPYNorm.y));
                    c.upperPositive[ev.NZ] = new Vector2d(-upperPXNorm.z, -upperPYNorm.z);
                    c.upperPositive[ev.upperPositiveX] = new Vector2d(1, 0);
                    c.upperPositive[ev.upperPositiveY] = new Vector2d(0, 1);
                    c.upperPositive.RHS = new Vector2d(uRHS, uRHS);

                    c.upperNegative[ev.NX] = -(c.upperNegative[ev.PX] = new Vector2d(upperNXNorm.x, upperNYNorm.x));
                    c.upperNegative[ev.NY] = -(c.upperNegative[ev.PY] = new Vector2d(upperNXNorm.y, upperNYNorm.y));
                    c.upperNegative[ev.NZ] = new Vector2d(-upperNXNorm.z, -upperNYNorm.z);
                    c.upperNegative[ev.upperNegativeX] = new Vector2d(1, 0);
                    c.upperNegative[ev.upperNegativeY] = new Vector2d(0, 1);
                    c.upperNegative.RHS = new Vector2d(uRHS, uRHS);
                }
            }

            float targetThrottle = vessel.ctrlState.mainThrottle;
            Vector3 thrustDirection = engineManager.thrustDirection;

            if (thrustDirection.sqrMagnitude < 1e-6) {
                goto end;
            }


            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                var ev = engineVariables[i];
                var c = engineConstraints[i];

                fuelProblem.Objective[ev.NZ] = -engine.maxFlow;
                if (engine.gimbalRange > 0) {
                    ev.PX.upperBound = double.PositiveInfinity;
                    ev.NX.upperBound = double.PositiveInfinity;
                    ev.PY.upperBound = double.PositiveInfinity;
                    ev.NY.upperBound = double.PositiveInfinity;

                    double r = Math.Sin(engine.gimbalRange * (Math.PI / 180));
                    double hscale = ((Math.Sqrt(1.0 - (2.0 * r * r)) * 0.5) - 0.5) / r;
                    double hflow = engine.maxFlow * hscale;
                    fuelProblem.Objective[ev.PX] = hflow;
                    fuelProblem.Objective[ev.NX] = hflow;
                    fuelProblem.Objective[ev.PY] = hflow;
                    fuelProblem.Objective[ev.NY] = hflow;

                    Vector3[] dir = new Vector3[3];
                    Vector3[] moments = new[] {
                        new Vector3(), new Vector3(), new Vector3()
                    };
                    double[] mag = new[] { 0.0, 0.0, 0.0 };

                    float thrust = engine.maxThrust / engine.NumThrustTransforms;

                    for (int j = 0; j < engine.NumThrustTransforms; j++) {
                        dir[0] = engine.getOrientation(j) * Vector3.left;
                        dir[1] = engine.getOrientation(j) * Vector3.down;
                        dir[2] = engine.getOrientation(j) * Vector3.back;

                        for (int k = 0; k < 3; k++) {
                            Vector3 force = thrust * dir[k];
                            Vector3 offset = engine.getPosition(j) - engine.vesselCoM;
                            Vector3 m = Vector3.Cross(offset, force);
                            mag[k] += Vector3.Dot(force, thrustDirection);
                            moments[k] += m;
                        }
                    }

                    thrustProblem.Objective[ev.PX] = mag[0];
                    thrustProblem.Objective[ev.NX] = -mag[0];
                    thrustProblem.Objective[ev.PY] = mag[1];
                    thrustProblem.Objective[ev.NY] = -mag[1];
                    thrustProblem.Objective[ev.NZ] = mag[2];

                    thrustMoment[ev.PX] = moments[0];
                    thrustMoment[ev.NX] = -moments[0];
                    thrustMoment[ev.PY] = moments[1];
                    thrustMoment[ev.NY] = -moments[1];
                    thrustMoment[ev.NZ] = moments[2];

                } else {
                    thrustProblem.Objective[ev.NZ] = engine.maxThrustMagnitude;
                    thrustMoment[ev.NZ] = engine.maxTorque;

                    ev.PX.upperBound = 0;
                    ev.NX.upperBound = 0;
                    ev.PY.upperBound = 0;
                    ev.NY.upperBound = 0;
                }
            }

            float detectionLimit = vessel.vesselSAS.controlDetectionThreshold * 0.5f;
            Vector3 angularInput = new Vector3(
                vessel.ctrlState.pitch,
                vessel.ctrlState.roll,
                vessel.ctrlState.yaw);
            Vector3 torqueInput = -vessel.transform.InverseTransformDirection(
                vessel.ReferenceTransform.rotation *
                angularInput);


            float torqueInputMagnitude = torqueInput.magnitude;
            bool torqueEnabled = false;

            Vector3d torqueDirection = Vector3d.zero;
            int i_axis = 0, d_axis1 = 0, d_axis2 = 0;
            double d_coeff1 = 0, d_coeff2 = 0;

            if (torqueInputMagnitude > 1e-4) {
                torqueEnabled = true;
                torqueDirection = torqueInput / torqueInputMagnitude;
                torqueInputMagnitude /= Mathf.Sqrt(3);

                Vector3d magDir = new Vector3d(
                    Math.Abs(torqueDirection.x),
                    Math.Abs(torqueDirection.y),
                    Math.Abs(torqueDirection.z));
                i_axis = magDir.x > magDir.y ? (magDir.x > magDir.z ? 0 : 2) : (magDir.y > magDir.z ? 1 : 2);

                d_axis1 = (i_axis + 1) % 3;
                d_axis2 = (i_axis + 2) % 3;
                d_coeff1 = torqueDirection[d_axis1] / torqueDirection[i_axis];
                d_coeff2 = torqueDirection[d_axis2] / torqueDirection[i_axis];

                foreach (var v in allVariables) {
                    torqueProblem.Objective[v] = Vector3d.Dot(
                        thrustMoment[v], torqueDirection);
                    torqueMoment[v] = new Vector2d(
                        thrustMoment[d_axis1][v] - d_coeff1 * thrustMoment[i_axis][v],
                        thrustMoment[d_axis2][v] - d_coeff2 * thrustMoment[i_axis][v]);

                }
            }
            thrustSolver.solve();

            if (thrustSolver.status != LP.Solver.Status.Optimal) {
                OSSDebug.Error("Thrust solver failure!: {0}", thrustSolver.status);
                goto end;
            }

            double maxThrust = thrustSolver.ObjectiveValue;

            double targetThrust = maxThrust * targetThrottle * 0.98f;

            torqueThrust.RHS = targetThrust;

            Vector3d targetTorque = Vector3.zero;
            double maxTorque = 0.0;
            if (torqueEnabled) {
                torqueSolver.solve();
                if (torqueSolver.status != LP.Solver.Status.Optimal) {
                    OSSDebug.Error("Torque solver failure!: {0}", torqueSolver.status);
                    goto end;
                }

                maxTorque = torqueSolver.ObjectiveValue;
                double targetTorqueMagnitude = torqueInputMagnitude * maxTorque * 0.98f;
                targetTorque = (targetTorqueMagnitude * torqueDirection) / torqueDirection.magnitude;
            }

            fuelMoment.RHS = targetTorque;

            fuelSolver.solve();

            if (fuelSolver.status != LP.Solver.Status.Optimal) {
                OSSDebug.Error("Fuel solver failure!: {0}", fuelSolver.status);
                goto end;
            }

            success = true;
            
        end:
            thrustSolver.finished();
            torqueSolver.finished();
            fuelSolver.finished();

            if (success) {
                var wheels = vessel.GetModules<WheelController>().Where(w => w.Operational);
                var torqueLeft = new Vector3d(fuelSolver[wheelX], fuelSolver[wheelY], fuelSolver[wheelZ]);
                for (int i = 0; i < wheels.Count(); i++) {
                    var wheel = wheels.ElementAt(i);

                    if (torqueLeft.x == 0) {
                        wheel.overrideInput.x = 0;
                    } else if (Math.Abs(torqueLeft.x) < wheel.torqueStrength.x) {
                        wheel.overrideInput.x = (float)(torqueLeft.x / wheel.torqueStrength.x);
                        torqueLeft.x = 0;
                    } else {
                        wheel.overrideInput.x = Mathf.Sign((float)torqueLeft.x);
                        torqueLeft.x -= wheel.overrideInput.x * wheel.torqueStrength.x;
                    }

                    if (torqueLeft.y == 0) {
                        wheel.overrideInput.y = 0;
                    } else if (Math.Abs(torqueLeft.y) < wheel.torqueStrength.y) {
                        wheel.overrideInput.y = (float)(torqueLeft.y / wheel.torqueStrength.y);
                        torqueLeft.y = 0;
                    } else {
                        wheel.overrideInput.y = Mathf.Sign((float)torqueLeft.y);
                        torqueLeft.y -= wheel.overrideInput.y * wheel.torqueStrength.y;
                    }

                    if (torqueLeft.z == 0) {
                        wheel.overrideInput.z = 0;
                    } else if (Math.Abs(torqueLeft.z) < wheel.torqueStrength.z) {
                        wheel.overrideInput.z = (float)(torqueLeft.z / wheel.torqueStrength.z);
                        torqueLeft.z = 0;
                    } else {
                        wheel.overrideInput.z = Mathf.Sign((float)torqueLeft.z);
                        torqueLeft.z -= wheel.overrideInput.z * wheel.torqueStrength.z;
                    }
                }
            }

            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                if (success) {
                    var ev = engineVariables[i];
                    var c = engineConstraints[i];

                    Vector3d thrust = new Vector3d(
                        fuelSolver[ev.PX] - fuelSolver[ev.NX],
                        fuelSolver[ev.PY] - fuelSolver[ev.NY],
                        -fuelSolver[ev.NZ]);
                    double mag = thrust.magnitude;
                    double rx = Math.Atan(thrust.y / thrust.z);
                    double ry = Math.Atan(thrust.x / (mag * Math.Cos(rx)));
                    if (mag > 1e-5) {
                        engine.requestedThrust = (float)mag * engine.maxThrust;
                        if (engine.gimbalRange > 0) {
                            engine.gimbalX = (float)(rx * (180.0 / Math.PI)) / engine.gimbalRange;
                            engine.gimbalY = (float)(ry * (180.0 / Math.PI)) / engine.gimbalRange;
                        } else {
                            engine.gimbalX = 0;
                            engine.gimbalY = 0;
                        }
                    } else {
                        engine.requestedThrust = vessel.ctrlState.mainThrottle * engine.maxThrust;
                        engine.gimbalX = 0;
                        engine.gimbalY = 0;
                    }
                } else {
                    engine.requestedThrust = vessel.ctrlState.mainThrottle * engine.maxThrust;
                    engine.gimbalX = 0;
                    engine.gimbalY = 0;
                }
            }

        }

        private Vector3 prevAngularVelocity = Vector3.zero;
        private Vector3 calculateCurrentTorque() {
            Vector3 angularVelocity = vessel.angularVelocity;
            Vector3 angularAcceleration = angularVelocity - prevAngularVelocity;
            return angularAcceleration * vessel.GetTotalMass();
        }

        public override void Load(ConfigNode node) { }

        public override void Save(ConfigNode node) { }

        public override void Start() {
            if (vessel) {
                engineManager = new EngineManager(vessel);
                engineManager.update();
                OSSDebug.Log("Balancing {0} Engines", engineManager.NumEngines);
                prevDampScalar = vessel.vesselSAS.dampingModeScalar;
            }
        }

        public override void Stop() {
            vessel.vesselSAS.dampingModeScalar = prevDampScalar;
        }

        public override void onStructuralChange() {
            if (vessel != null) {
                engineManager.update();
            }
        }
        public override void onGUI() { }

        public override IEnumerable<PartController> controlledParts() {
            IEnumerable<PartController> parts = new PartController[0];
            if (engineManager != null) {
                parts = engineManager.Engines.Cast<PartController>();
            }
            return parts.Concat(
                vessel.GetModules<WheelController>().Cast<PartController>());
        }

        private class EngineVariableSet {
            public LP.BoundedVariable NZ, PX, NX, PY, NY;
            public LP.BoundedVariable lowerNegativeX, lowerPositiveX, lowerNegativeY, lowerPositiveY;
            public LP.BoundedVariable upperNegativeX, upperPositiveX, upperNegativeY, upperPositiveY;

            public EngineVariableSet(EngineController e) {
                int index = e.index;
                if (e.throttleLocked) {
                    NZ = new LP.BoundedVariable(1, 1);
                } else {
                    double lower = 5e-4;
                    if (e.vessel.ctrlState.mainThrottle < lower) lower = 0;
                    NZ = new LP.BoundedVariable(lower, 1);
                }
                NZ.Name = "NZ " + index;
                PX = new LP.BoundedVariable();
                PX.Name = "PX " + index;
                NX = new LP.BoundedVariable();
                NX.Name = "NX " + index;
                PY = new LP.BoundedVariable();
                PY.Name = "PY " + index;
                NY = new LP.BoundedVariable();
                NY.Name = "NY " + index;

                lowerNegativeX = new LP.BoundedVariable();
                lowerPositiveX = new LP.BoundedVariable();
                lowerNegativeY = new LP.BoundedVariable();
                lowerPositiveY = new LP.BoundedVariable();
                upperNegativeX = new LP.BoundedVariable();
                upperPositiveX = new LP.BoundedVariable();
                upperNegativeY = new LP.BoundedVariable();
                upperPositiveY = new LP.BoundedVariable();

                lowerNegativeX.Name = "lowerNegativeX " + index;
                lowerPositiveX.Name = "lowerPositiveX " + index;
                lowerNegativeY.Name = "lowerNegativeY " + index;
                lowerPositiveY.Name = "lowerPositiveY " + index;
                upperNegativeX.Name = "upperNegativeX " + index;
                upperPositiveX.Name = "upperPositiveX " + index;
                upperNegativeY.Name = "upperNegativeY " + index;
                upperPositiveY.Name = "upperPositiveY " + index;
            }

            public IEnumerable<LP.Variable> all() {
                return new[] {
                    NZ, PX, NX, PY, NY,
                    lowerNegativeX, lowerPositiveX, lowerNegativeY, lowerPositiveY,
                    upperNegativeX, upperPositiveX, upperNegativeY, upperPositiveY
                };
            }
        }

        private class EngineConstraintSet : LP.ConstraintSet {
            private LP.BiConstraint[] constraints;

            public LP.BiConstraint lowerNegative {
                get {
                    return constraints[0];
                }
            }

            public LP.BiConstraint lowerPositive {
                get {
                    return constraints[1];
                }
            }

            public LP.BiConstraint upperNegative {
                get {
                    return constraints[2];
                }
            }

            public LP.BiConstraint upperPositive {
                get {
                    return constraints[3];
                }
            }

            public EngineConstraintSet() {
                constraints = new[]{
                    new LP.BiConstraint(),
                    new LP.BiConstraint(),
                    new LP.BiConstraint(),
                    new LP.BiConstraint()
                };
            }

            protected override IEnumerable<LP.Constraint> GetConstraints() {
                return constraints.SelectMany(x => x);
            }
        }
    }
}

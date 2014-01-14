using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS.ControlMode {
    class BalanceShip : ControlMode {
        private EngineManager engineManager;

        public override Vessel vessel { get; set; }
        
        public override void onFixedUpdate() {
            if (engineManager == null) return;
            engineManager.update();

            var numEngines = engineManager.OperationalEngines.Count();

            if (numEngines == 0) return;

            bool success = true;

            Vector3 currentTorque = calculateCurrentTorque();

            var thrustSolver = new LP.MaxSolver();
            var thrustProblem = thrustSolver.Problem;

            Vector3 wheelTorque = Vector3.zero;
            foreach (var wheel in vessel.GetModules<WheelController>().Where(w => w.Operational)) {
                wheelTorque += wheel.torqueStrength;
            }

            // Create variables
            EngineVariableSet[] engineVariables = engineManager.OperationalEngines.Select(e => new EngineVariableSet(e)).ToArray();
            
            LP.Variable[] allVariables =
                engineVariables.SelectMany(ev => ev.all())
                .ToArray();

            thrustSolver.Variables = allVariables;

            // Create Constraints
            var thrustMoment = new LP.TriConstraint();
            thrustProblem.addConstraint(thrustMoment);

            
            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                var ev = engineVariables[i];

                LP.Constraint 
                    ny = new LP.Constraint(), 
                    py = new LP.Constraint(),
                    nx = new LP.Constraint(),
                    px = new LP.Constraint();

                ny.type = LP.Constraint.Type.GE;
                py.type = LP.Constraint.Type.GE;
                nx.type = LP.Constraint.Type.GE;
                px.type = LP.Constraint.Type.GE;

                thrustProblem.addConstraint(new []{ ny, py, nx, px });

                Vector3d origin = Vector3d.zero;
                Vector3d apo = Vector3d.back;

                double range = engine.gimbalRange * (Math.PI / 180.0);
                double sin_t = Math.Sin(range);
                double cos_t = Math.Cos(range);

                ny[ev.Y] = -cos_t;
                ny[ev.Z] = -sin_t;

                py[ev.Y] = cos_t;
                py[ev.Z] = -sin_t;

                px[ev.X] = cos_t;
                px[ev.Z] = -sin_t;

                nx[ev.X] = -cos_t;
                nx[ev.Z] = -sin_t;

                LP.Constraint
                    tny = new LP.Constraint(),
                    tpy = new LP.Constraint(),
                    tnx = new LP.Constraint(),
                    tpx = new LP.Constraint();
                
                tny.type = LP.Constraint.Type.LE;
                tpy.type = LP.Constraint.Type.LE;
                tnx.type = LP.Constraint.Type.LE;
                tpx.type = LP.Constraint.Type.LE;

                thrustProblem.addConstraint(new[] { tny, tpy, tnx, tpx });

                tny[ev.Y] = -sin_t;
                tny[ev.Z] = -cos_t;
                tny.RHS = engine.maxThrust * cos_t;

                tpy[ev.Y] = sin_t;
                tpy[ev.Z] = -cos_t;
                tpy.RHS = engine.maxThrust * cos_t;

                tnx[ev.X] = -sin_t;
                tnx[ev.Z] = -cos_t;
                tnx.RHS = engine.maxThrust * cos_t;

                tpx[ev.X] = sin_t;
                tpx[ev.Z] = -cos_t;
                tpx.RHS = engine.maxThrust * cos_t;
            }

            Vector3 thrustDirection = engineManager.thrustDirection;

            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                var ev = engineVariables[i];

                if (engine.gimbalRange > 0) {

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

                    thrustProblem.Objective[ev.X] = mag[0];
                    thrustProblem.Objective[ev.Y] = mag[1];
                    thrustProblem.Objective[ev.Z] = -1;

                    thrustMoment[ev.X] = moments[0] / -engine.maxThrust;
                    thrustMoment[ev.Y] = moments[1] / -engine.maxThrust;
                    thrustMoment[ev.Z] = moments[2] / -engine.maxThrust;
                } else {
                    thrustProblem.Objective[ev.Z] = -1;
                    thrustMoment[ev.Z] = -engine.maxTorque / engine.maxThrust;
                }
            }

            thrustSolver.solve();

            if (thrustSolver.status != LP.Solver.Status.Optimal) {
                OSSDebug.Error("Thrust solver failure!: {0}", thrustSolver.status);
                success = false;
            }
            thrustSolver.dumpState();

            for (int i = 0; i < numEngines; i++) {
                var engine = engineManager.OperationalEngines.ElementAt(i);
                if (success) {
                    var ev = engineVariables[i];

                    Vector3d thrust = new Vector3d(
                        -thrustSolver[ev.X],
                        -thrustSolver[ev.Y],
                        thrustSolver[ev.Z]) * vessel.ctrlState.mainThrottle;
                    Vector3 offset = engine.getPosition(0) - engine.vesselCoM;
                    OSSDebug.Log("#{0} Torque: {1}", engine.index, Vector3.Cross(
                        engine.getOrientation(0) * offset, thrust));
                    double mag = thrust.magnitude;
                    double rx = Math.Atan(thrust.y / thrust.z);
                    double ry = Math.Atan(thrust.x / (mag * Math.Cos(rx)));
                    OSSDebug.Log("#{0} - Mag: {1:F3}, RX: {2:F3}, RY: {3:F3}", engine.index,
                        mag, rx * 180.0 / Math.PI, ry * 180.0 / Math.PI);
                    if (mag > 1e-5) {
                        engine.requestedThrust = (float)mag;
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

            thrustSolver.finished();
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
            }
        }

        public override void Stop() {
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
            public LP.Variable X, Y, Z;

            public EngineVariableSet(EngineController e) {
                int index = e.index;

                Z = new LP.BoundedVariable(-e.maxThrust, 0);
                Z.Name = "Z " + index;
                X = new LP.FreeVariable();
                X.Name = "X " + index;
                Y = new LP.FreeVariable();
                Y.Name = "Y " + index;
            }

            public IEnumerable<LP.Variable> all() {
                return new[] {
                    X, Y, Z
                };
            }
        }
    }
}

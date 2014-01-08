using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS {
    public class EngineManager {

        public Vessel vessel { get; private set; }
        private EngineController[] engines;

        public int NumEngines { get; private set; }
        public Vector3 thrustDirection { get; private set; }
        
        public IEnumerable<EngineController> OperationalEngines {
            get {
                return engines.Where(e => e.Operational);
            }
        }

        public IEnumerable<EngineController> Engines {
            get {
                return engines;
            }
        }

        public EngineManager(Vessel vessel) {
            this.vessel = vessel;
        }

        public void update() {
            updateEngines();

            Vector3 CoM = vessel.transform.InverseTransformPoint(vessel.CoM + vessel.rb_velocity * Time.deltaTime);

            thrustDirection = Vector3.zero;
            Vector3 maxTorque = Vector3.zero;
            foreach (var engine in OperationalEngines) {
                engine.vesselCoM = CoM;
                engine.update();
                thrustDirection += engine.maxThrustForce;
                maxTorque += engine.maxTorque;
            }

            thrustDirection = thrustDirection.normalized;

            foreach (var engine in OperationalEngines) {
                engine.updateMagnitude(thrustDirection);
            }
        }

        private void updateEngines() {
            engines = vessel.GetModules<EngineController>().ToArray();
            NumEngines = engines.Count();

            int i = 1;
            foreach (var engine in engines) {
                engine.index = i++;
            }
        }
        
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS {
    public class WheelController : PartController {

        private bool isActive = false;
        private ModuleReactionWheel wheel;

        private Vector3 ctrlInput = Vector3.zero;
        public Vector3 overrideInput = Vector3.zero;
        private double[] rates = new double[0];

        private Vector3 _torqueStrength = Vector3.zero;
        public Vector3 torqueStrength {
            get {
                return _torqueStrength;
            }
            private set {
                _torqueStrength = value;
            }
        }

        public bool Operational { get; private set; }

        public override void OnStart(StartState state) {
            base.OnStart(state);

            if (vessel == null) return;

            wheel = vessel.GetModules<ModuleReactionWheel>().First();
            torqueStrength = new Vector3(wheel.PitchTorque, wheel.RollTorque, wheel.YawTorque);

            //wheel.PitchTorque = 0;
            //wheel.RollTorque = 0;
            //wheel.YawTorque = 0;

            isActive = true;

            rates = new double[wheel.inputResources.Count];
            for (int i = 0; i < wheel.inputResources.Count; i++) {
                rates[i] = wheel.inputResources[i].rate;
                wheel.inputResources[i].rate = 1e-5f;
            }
        }

        public override void OnUpdate() {
            base.OnUpdate();
            if (vessel == null) return;
            if (!isActive) {
                torqueStrength = new Vector3(wheel.PitchTorque, wheel.RollTorque, wheel.YawTorque);
                isActive = true;
            }
            ctrlInput = new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw);
        }

        public override void FixedUpdate() {
            base.FixedUpdate();
            if (wheel == null) return;
            if (vessel == null) return;
            if (vessel.rigidbody == null) return;

            Operational = wheel.State == ModuleReactionWheel.WheelState.Active && wheel.inputResources.All(r => r != null && r.available);
            if (!Operational) return;

            Vector3 input = controlState == ControlState.Local ? ctrlInput : overrideInput;
            double requiredResources = Math.Abs(input.x) + Math.Abs(input.y) + Math.Abs(input.z);

            requiredResources *= TimeWarp.deltaTime;
            bool hasPower = true;

            if (!CheatOptions.InfiniteFuel) {
                for (int i = 0; i < wheel.inputResources.Count; i++) {
                    var resource = wheel.inputResources[i];
                    if (resource == null) continue;
                    var rate = rates[i];
                    var requestedResource = rate * requiredResources;

                    double supply = part.RequestResource(resource.id, requestedResource);
                    if (supply < requestedResource * 0.9) {
                        hasPower = false;
                        break;
                    }
                }
            }
            if (hasPower) {
                Vector3 torque = vessel.ReferenceTransform.rotation * Vector3.Scale(input, -torqueStrength);
                vessel.rigidbody.AddTorque(torque, ForceMode.Force);
                Vector3 compensation = vessel.ReferenceTransform.rotation * Vector3.Scale(ctrlInput, torqueStrength);
                vessel.rigidbody.AddTorque(compensation, ForceMode.Force);
            }
        }

    }
}

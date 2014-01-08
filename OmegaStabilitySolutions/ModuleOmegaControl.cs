using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;

namespace OmegaSS {

    [KSPModule("Omega Stability System")]
    public class ModuleOmegaControl : PartModule {

        private ControlMode.ControlMode controlMode;
        private bool inControl = true;
        private bool inEditor = true;

        private int prevPartCount = 0;
        public bool VesselChanged {
            get { return prevPartCount != vessel.Parts.Count; }
        }

        [Persistent]
        private bool activated = true;

        public override void OnStart(PartModule.StartState state) {
            OSSDebug.Log("Starting Omega Stability System");
            if (state != StartState.Editor && state != StartState.None) {
                inEditor = false;
                if (activated)
                    ActivateSystem();

            } else {
                inEditor = true;
            }
            RenderingManager.AddToPostDrawQueue(3, new Callback(OnGUI));
            base.OnStart(state);
        }

        public void FixedUpdate() {
            if (inEditor) return;
            if (VesselChanged) onStructuralChange();
            if (!inControl) return;

            if (controlMode == null) return;

            controlMode.onFixedUpdate();
        }

        private void onStructuralChange() {
            prevPartCount = vessel.Parts.Count;
            determineControl();
            if (inControl) {
                var allParts = vessel.GetModules<PartController>();
                if (controlMode != null) {
                    controlMode.onStructuralChange();

                    var controlledParts = controlMode.controlledParts();

                    foreach (var part in controlledParts) {
                        part.setController(this, () => this.vessel);
                    }
                    foreach (var part in allParts.Except(controlledParts)) {
                        part.setLocalControl();
                    }
                } else {
                    foreach (var part in allParts) {
                        part.setLocalControl();
                    }
                }
                Events["DeactivateSystem"].active = activated;
                Events["ActivateSystem"].active = !activated;
                Actions["ToggleSystem"].active = true;
            } else {
                Events["DeactivateSystem"].active = false;
                Events["ActivateSystem"].active = false;
                Actions["ToggleSystem"].active = false;
            }
        }

        private void determineControl() {
            var allControllers = vessel.GetModules<ModuleOmegaControl>();
            var hadControl = allControllers.Where(c => c.inControl);

            switch (hadControl.Count()) {
                case 0: 
                    inControl = true;
                    break;
                default:
                    bool first = true;
                    foreach (var m in hadControl) {
                        if (first) {
                            m.inControl = true;
                            first = false;
                        } else {
                            m.inControl = false;
                        }
                    }
                    break;
            }

        }

        public void OnGUI() {
            if (inControl && controlMode != null && !inEditor) {
                GUI.skin = HighLogic.Skin;
                controlMode.onGUI();
            }
        }

        public override string GetInfo() {
            return "Equipped";
        }

        [KSPEvent(guiName = "Activate Omega Stability", guiActive = true, guiActiveEditor = true, active = true)]
        public void ActivateSystem() {
            activated = true;
            if (!inEditor) {
                if (!inControl) return;
                controlMode = new ControlMode.BalanceShip();
                controlMode.vessel = vessel;
                controlMode.Start();
            }
            Events["DeactivateSystem"].active = true;
            Events["ActivateSystem"].active = false;

            onStructuralChange();
        }

        [KSPEvent(guiName = "Deactivate Omega Stability", guiActive = true, guiActiveEditor = true, active = false)]
        public void DeactivateSystem() {
            activated = false;
            if (!inEditor) {
                if (!inControl) return;
                controlMode.Stop();
                controlMode = null;
            }
            Events["DeactivateSystem"].active = false;
            Events["ActivateSystem"].active = true;

            onStructuralChange();
        }

        [KSPAction("ToggleSystem", guiName = "Toggle Omega Stability")]
        public void ToggleSystem(KSPActionParam param) {
            if (!inControl) return;
            if (param.type == KSPActionType.Activate)
                ActivateSystem();
            else
                DeactivateSystem();
        }
    }

    public static class VesselExtensions {
        public static IEnumerable<T> GetModules<T>(this Vessel vessel) where T : PartModule {
            return vessel.parts.SelectMany(p => p.Modules.OfType<T>());
        }
    }
}

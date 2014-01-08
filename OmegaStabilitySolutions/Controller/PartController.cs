using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace OmegaSS {
    public abstract class PartController : PartModule {
        public enum ControlState {
            Local,
            External
        }

        protected ControlState controlState = ControlState.Local;

        [KSPField(guiName = "Control state", guiActive=true, isPersistant=false)]
        public string ControlString;

        private void updateControlString() {
            ControlString = controlState.ToString();
        }

        public ControlState Control {
            get { return controlState; }
        }

        protected object controller = null;
        public delegate Vessel GetVessel();
        protected GetVessel getControllerVessel = null;
        public object Controller {
            get { return controller; }
        }

        public Vessel ControlVessel {
            get {
                try {
                    return getControllerVessel();
                } catch (Exception) {
                    return null;
                }
            }
        }

        public void setLocalControl() {
            controlState = ControlState.Local;
            controller = null;
            getControllerVessel = null;
        }

        public void setController(object value, GetVessel callback) {
            controlState = ControlState.External;
            controller = value;
            getControllerVessel = callback;
        }

        public virtual void FixedUpdate() {
            if (controlState == ControlState.External && ControlVessel != vessel)
                setLocalControl();
            updateControlString();
        }
    }
}

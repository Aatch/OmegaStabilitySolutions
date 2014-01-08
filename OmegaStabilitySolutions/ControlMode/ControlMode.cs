using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS.ControlMode {
    public abstract class ControlMode : IConfigNode {
        public abstract Vessel vessel { get; set; }

        public abstract void Load(ConfigNode node);
        public abstract void Save(ConfigNode node);

        public abstract void Start();
        public abstract void Stop();

        public abstract void onStructuralChange();
        public abstract void onFixedUpdate();
        public abstract void onGUI();

        public abstract IEnumerable<PartController> controlledParts();
    }
}

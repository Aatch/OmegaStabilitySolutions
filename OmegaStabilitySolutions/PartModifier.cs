using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS {
    [KSPAddonFixed(KSPAddon.Startup.Instantly, true, typeof(PartModifier))]
    public class PartModifier : MonoBehaviour {

        private static bool loaded = false;
        public PartModifier() {
            if (loaded) return;
            addPartModules();
            loaded = true;
        }

        private void addPartModules() {
            foreach (var urlconfig in GameDatabase.Instance.root.AllConfigs) {
                if (urlconfig.config.name == "PART") {
                    var config = urlconfig.config;
                    List<ConfigNode> engineConfigs = new List<ConfigNode>();
                    List<string> transformNames = new List<string>();
                    bool thrustTransform = false;

                    foreach (var node in config.GetNodes("MODULE")) {
                        var name = node.GetValue("name");

                        if (name == typeof(ModuleEngines).Name ||
                            name == typeof(ModuleEnginesFX).Name) {
                                engineConfigs.Add(node);
                        } else if (name == typeof(ModuleGimbal).Name) {
                            if (node.HasValue("gimbalTransformName"))
                                transformNames.Add(node.GetValue("gimbalTransformName"));
                            else
                                thrustTransform = true;
                        } else if (name == typeof(ModuleReactionWheel).Name) {
                            ConfigNode aug = config.AddNode("MODULE");
                            aug.AddValue("name", typeof(WheelController).Name);
                        }

                    }
                    foreach (var engine in engineConfigs) {
                        bool gimballed;
                        string tname;

                        if (engine.HasValue("thrustVectorTransformName")) {
                            tname = engine.GetValue("thrustVectorTransformName");
                            gimballed = transformNames.Contains(tname);
                        } else {
                            tname = "thrustTransform";
                            gimballed = thrustTransform;
                        }

                        ConfigNode aug = config.AddNode("MODULE");
                        aug.AddValue("name", "EngineController");
                        aug.AddValue("thrustVectorTransformName", tname);
                        aug.AddValue("hasGimbal", gimballed ? "true" : "false");
                    }
                }
            }
        }

    }
}

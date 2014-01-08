using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS {

    public interface IEngineInfo {

        float g { get; }
        /// <summary>
        /// Whether or not this engine is operational
        /// </summary>
        bool isOperational { get; }
        bool isRunningEngine { get; }
        
        #region Thrust Information
        /// <summary>
        /// The current maximum thrust of this engine
        /// </summary>
        float maxThrust { get; }
        /// <summary>
        /// The current minimum thrust of this engine
        /// </summary>
        float minThrust { get; }
        float thrustPercentage { get; set; }
        float realIsp { get; set; }
        bool flameout { get; }
        bool EngineIgnited { get; }
        bool engineShutdown { get; }
        float requestedThrust { get; }
        FloatCurve atmosphereCurve { get; }
        #endregion

        #region Throttle Information
        /// <summary>
        /// The current throttle setting of this engine in the range [0:1]
        /// </summary>
        float currentThrottle { get; set; }
        float requestedThrottle { get; }
        bool throttleLocked { get; set; }
        /// <summary>
        /// Whether or not this engine will respond to thrust changes instantly
        /// </summary>
        bool useEngineResponseTime { get; set; }
        /// <summary>
        /// The speed at which the engine can increase thrust
        /// </summary>
        float engineAccelerationSpeed { get; set; }
        /// <summary>
        /// The speed at which the engine can decrese thrust
        /// </summary>
        float engineDecelerationSpeed { get; set; }
        #endregion

        IEnumerable<Transform> Transforms();
    }

    public class ModuleEnginesInfo : IEngineInfo {
        private ModuleEngines engine;

        public ModuleEnginesInfo(ModuleEngines engine) {
            this.engine = engine;
        }

        public bool isOperational {
            get {
                return engine.isOperational;
            }
        }

        public float maxThrust {
            get { return engine.maxThrust; }
        }

        public float minThrust {
            get { return engine.minThrust; }
        }

        public bool throttleLocked {
            get { return engine.throttleLocked; }
            set { engine.throttleLocked = value; }
        }

        public float currentThrottle {
            get { return engine.currentThrottle; }
            set { engine.currentThrottle = value; }
        }

        public bool useEngineResponseTime {
            get {
                return engine.useEngineResponseTime;
            }
            set {
                engine.useEngineResponseTime = value;
            }
        }

        public float engineAccelerationSpeed {
            get {
                return engine.engineAccelerationSpeed;
            }
            set {
                engine.engineAccelerationSpeed = value;
            }
        }

        public float engineDecelerationSpeed {
            get {
                return engine.engineDecelerationSpeed;
            }
            set {
                engine.engineDecelerationSpeed = value;
            }
        }

        public IEnumerable<Transform> Transforms() {
            return engine.thrustTransforms;
        }


        public float thrustPercentage {
            get {
                return engine.thrustPercentage;
            }
            set {
                engine.thrustPercentage = value;
            }
        }

        public float realIsp {
            get {
                return engine.realIsp;
            }
            set {
                engine.realIsp = value;
            }
        }

        public float g {
            get {
                return engine.g;
            }
        }

        public bool flameout {
            get { return engine.flameout; }
        }

        public bool EngineIgnited {
            get { return engine.EngineIgnited; }
        }

        public bool engineShutdown {
            get { return engine.engineShutdown; }
        }


        public float requestedThrust {
            get { return engine.requestedThrust; }
        }

        public FloatCurve atmosphereCurve {
            get { return engine.atmosphereCurve; }
        }

        public float requestedThrottle {
            get { return engine.requestedThrottle; }
        }

        public bool isRunningEngine { get { return true; } }
    }

    public class ModuleEnginesFXInfo : IEngineInfo {
        private ModuleEnginesFX engine;
        private MultiModeEngine mme;

        public ModuleEnginesFXInfo(ModuleEnginesFX engine) {
            this.engine = engine;
            this.mme = engine.part.Modules.OfType<MultiModeEngine>().FirstOrDefault();
        }

        public bool isOperational {
            get {
                return engine.isOperational;
            }
        }

        public float maxThrust {
            get { return engine.maxThrust; }
        }

        public float minThrust {
            get { return engine.minThrust; }
        }

        public bool throttleLocked {
            get { return engine.throttleLocked; }
            set { engine.throttleLocked = value; }
        }

        public float currentThrottle {
            get { return engine.currentThrottle; }
            set { engine.currentThrottle = value; }
        }

        public bool useEngineResponseTime {
            get {
                return engine.useEngineResponseTime;
            }
            set {
                engine.useEngineResponseTime = value;
            }
        }

        public float engineAccelerationSpeed {
            get {
                return engine.engineAccelerationSpeed;
            }
            set {
                engine.engineAccelerationSpeed = value;
            }
        }

        public float engineDecelerationSpeed {
            get {
                return engine.engineDecelerationSpeed;
            }
            set {
                engine.engineDecelerationSpeed = value;
            }
        }

        public IEnumerable<Transform> Transforms() {
            return engine.thrustTransforms;
        }


        public float thrustPercentage {
            get {
                return engine.thrustPercentage;
            }
            set {
                engine.thrustPercentage = value;
            }
        }

        public float realIsp {
            get {
                return engine.realIsp;
            }
            set {
                engine.realIsp = value;
            }
        }

        public float g {
            get {
                return engine.g;
            }
        }

        public bool flameout {
            get { return engine.flameout; }
        }

        public bool EngineIgnited {
            get { return engine.EngineIgnited; }
        }

        public bool engineShutdown {
            get { return engine.engineShutdown; }
        }

        public float requestedThrust {
            get { return engine.requestedThrust; }
        }

        public FloatCurve atmosphereCurve {
            get { return engine.atmosphereCurve; }
        }

        public float requestedThrottle {
            get { return engine.requestedThrottle; }
        }

        public bool isRunningEngine {
            get {
                return mme.runningPrimary && mme.primaryEngineID == engine.engineID;
            }
        }
    }

}

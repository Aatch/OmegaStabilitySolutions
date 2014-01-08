using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OmegaSS {
    [KSPModule("Omega Engine Stabilizer")]
    public class EngineController : PartController {

        [KSPField(isPersistant = false)]
        public string thrustVectorTransformName = "thrustTransform";

        [KSPField(isPersistant = false)]
        public bool hasGimbal = false;

        [KSPField(isPersistant = false)]
        public string engineID = null;

        public bool Started { get; private set; }

        private ModuleGimbal gimbal = null;
        public IEngineInfo Engine { get; private set; }

        public Vector3 vesselCoM = Vector3.zero;

        public int index = 0;

        public bool throttleLocked {
            get {
                return Engine.throttleLocked;
            }
        }
        public float maxThrust {
            get {
                return Engine.minThrust + ((Engine.maxThrust - Engine.minThrust) * (Engine.thrustPercentage / 100));
            }
        }
        public bool useEngineResponseTime { get; private set; }
        public float engineAccelerationSpeed { get; private set; }
        public float engineDecelerationSpeed { get; private set; }

        public float gimbalRange {
            get {
                if (!hasGimbal) return 0;
                return gimbal.gimbalLock ? 0 : gimbal.gimbalRange;
            }

        }

        public float requestedThrust { get; set; }
        public float currentThrottle {
            get {
                return Engine.currentThrottle;
            }
        }
        public float gimbalX { get; set; }
        public float gimbalY { get; set; }
        
        /// <summary>
        /// Direction of thrust this engine produces as a normalized vector
        /// </summary>
        public Vector3 thrustDirection { get; private set; }

        /// <summary>
        /// Maximum amount of thrust this engine can produce, as a vector
        /// </summary>
        public Vector3 maxThrustForce { get; private set; }
        /// <summary>
        /// Maximum amount of torque this engine can produce at the current gimbal
        /// </summary>
        public Vector3 maxTorque { get; private set; }

        /// <summary>
        /// Maximum thrust magnitude for the engine
        /// </summary>
        public float maxThrustMagnitude { get; private set; }

        public float maxFlow {
            get {
                float flow = maxThrust / (Engine.realIsp * Engine.g);
                if (float.IsNaN(flow)) return 0;
                else return flow;
            }
        }

        private Transform[] initTransforms;
        private List<Transform> thrustTransforms = new List<Transform>();

        public int NumThrustTransforms { get { return thrustTransforms.Count; } }

        private StartState startState;

        public bool Operational {
            get {
                return Started && Engine.EngineIgnited && !Engine.flameout && !Engine.engineShutdown;
            }
        }

        public override void OnStart(StartState state) {
            startState = state;
            base.OnStart(state);

            if (engineID != null && engineID.Length > 0) {
                Engine = new ModuleEnginesFXInfo(part.Modules.OfType<ModuleEnginesFX>()
                    .Where(m => m.engineID == engineID).First());
            } else {
                Engine = new ModuleEnginesInfo(part.Modules.OfType<ModuleEngines>()
                    .Where(m => m.thrustVectorTransformName == thrustVectorTransformName)
                    .First());
            }

            useEngineResponseTime = Engine.useEngineResponseTime;
            if (useEngineResponseTime) {
                engineAccelerationSpeed = Engine.engineAccelerationSpeed;
                engineDecelerationSpeed = Engine.engineDecelerationSpeed;
            }
            Engine.useEngineResponseTime = true;
            Engine.engineAccelerationSpeed = 0.0f;
            Engine.engineDecelerationSpeed = 0.0f;

            requestedThrust = Engine.requestedThrust;

            if (hasGimbal) {

                gimbal = part.Modules.OfType<ModuleGimbal>()
                    .Where(m => m.gimbalTransformName == thrustVectorTransformName)
                    .First();

                initTransforms = new Transform[gimbal.gimbalTransforms.Count];
                List<Quaternion> initRots = (List<Quaternion>)(typeof(ModuleGimbal).GetField("initRots", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).GetValue(gimbal));

                int i = 0;
                foreach (var t in gimbal.gimbalTransforms) {
                    t.localRotation = initRots[i];
                    thrustTransforms.Add(t);

                    var obj = new GameObject();
                    var objTransform = obj.transform;
                    objTransform.parent = t.parent;
                    objTransform.localRotation = t.localRotation;
                    objTransform.localPosition = t.localPosition;
                    objTransform.localScale = t.localScale;

                    initTransforms[i] = objTransform;

                    i++;
                }

                gimbal.gimbalTransforms.Clear();
            } else {
                foreach (var t in Engine.Transforms()) {
                    thrustTransforms.Add(t);
                }
            }

            Started = true;
        }

        public override void FixedUpdate() {
            base.FixedUpdate();

            if (Operational && startState != StartState.Editor) {
                Engine.realIsp = Engine.atmosphereCurve.Evaluate((float)vessel.staticPressure);
                if (controlState == ControlState.Local) {
                    doLocalControl();
                } else {
                    doExternalControl();
                }
            }
        }

        private void doLocalControl() {
            float throttle = Engine.requestedThrottle;
            float targetX, targetY;
            if (gimbalRange > 0) {
                Vector3 torqueDirX = Vector3.zero;
                Vector3 torqueDirY = Vector3.zero;
                float angleRadians = gimbalRange * (Mathf.PI / 180);
                float range = Mathf.Sin(angleRadians);

                foreach (var t in thrustTransforms) {
                    Vector3 pxUnitForce = transformDirection(
                        t.rotation * Quaternion.AngleAxis(gimbalRange, Vector3.right) * -Vector3.forward);
                    Vector3 nxUnitForce = transformDirection(
                        t.rotation * Quaternion.AngleAxis(-gimbalRange, Vector3.right) * -Vector3.forward);
                    Vector3 pyUnitForce = transformDirection(
                        t.rotation * Quaternion.AngleAxis(gimbalRange, Vector3.up) * -Vector3.forward);
                    Vector3 nyUnitForce = transformDirection(
                        t.rotation * Quaternion.AngleAxis(-gimbalRange, Vector3.up) * -Vector3.forward);

                    Vector3 offset = transformPoint(t.position) - vesselCoM;

                    Vector3 xUnitMoment = Vector3.Cross(offset, pxUnitForce);
                    Vector3 nxUnitMoment = Vector3.Cross(offset, nxUnitForce);
                    Vector3 yUnitMoment = Vector3.Cross(offset, pyUnitForce);
                    Vector3 nyUnitMoment = Vector3.Cross(offset, nyUnitForce);

                    Vector3 xDiff = xUnitMoment - nxUnitMoment;
                    Vector3 yDiff = yUnitMoment - nyUnitMoment;

                    torqueDirX += xDiff;
                    torqueDirY += yDiff;
                }

                torqueDirX.Normalize();
                torqueDirY.Normalize();

                Vector3 target = targetInput();
                Vector3 targetNorm = target.normalized;

                Vector2 projected = new Vector2(Vector3.Dot(target, torqueDirX), Vector3.Dot(target, torqueDirY));
                Vector2 projectedNorm = new Vector2(Vector3.Dot(targetNorm, torqueDirX), Vector3.Dot(targetNorm, torqueDirY));

                if (projectedNorm.magnitude > 1e-2) projected /= projectedNorm.magnitude;

                targetX = projected.x;
                targetY = projected.y;
            } else {
                targetX = 0;
                targetY = 0;
            }

            setEngineValues(throttle, targetX, targetY);
        }

        private void doExternalControl() {
            setEngineValues(requestedThrust / maxThrust, gimbalX, gimbalY);
        }

        private void setEngineValues(float throttle, float targetX, float targetY) {
            if (float.IsNaN(throttle))
                throttle = 0;
            if (float.IsNaN(targetX))
                targetX = 0;
            if (float.IsNaN(targetY))
                targetY = 0;

            throttle = Mathf.Clamp(throttle, 0, 1);
            targetX = Mathf.Clamp(targetX, -1, 1);
            targetY = Mathf.Clamp(targetY, -1, 1);

            if (initTransforms != null) {
                int i = 0;
                foreach (var t in thrustTransforms) {
                    if (gimbalRange == 0) {
                        t.localRotation = initTransforms[i].localRotation;
                    } else {
                        t.localRotation = initTransforms[i].localRotation *
                            Quaternion.AngleAxis(targetY * gimbalRange, Vector3.up) *
                            Quaternion.AngleAxis(targetX * gimbalRange, Vector3.right);
                    }

                    i++;
                }
            }

            if (!throttleLocked) {
                if (useEngineResponseTime) {
                    if (Engine.currentThrottle > throttle)
                        Engine.currentThrottle = Mathf.Lerp(Engine.currentThrottle, throttle, engineDecelerationSpeed * TimeWarp.fixedDeltaTime);
                    else
                        Engine.currentThrottle = Mathf.Lerp(Engine.currentThrottle, throttle, engineAccelerationSpeed * TimeWarp.fixedDeltaTime);
                } else {
                    Engine.currentThrottle = throttle;
                }
            } else {
                Engine.currentThrottle = 1f;
            }

            Engine.currentThrottle = Mathf.Clamp(Engine.currentThrottle, 0f, 1f);
        }
        
        private Vector3 targetInput() {
            return -transformDirection(vessel.ReferenceTransform.rotation *
                new Vector3(vessel.ctrlState.pitch, vessel.ctrlState.roll, vessel.ctrlState.yaw));
        }

        private Vector3 transformDirection(Vector3 v) {
            return vessel.transform.InverseTransformDirection(v);
        }

        private Vector3 transformPoint(Vector3 v) {
            return vessel.transform.InverseTransformPoint(v);
        }

        public void update() {
            if (!Started) return;
            updateThrustInformation();
        }

        public void updateMagnitude(Vector3 overallThrustDirection) {
            maxThrustMagnitude = Vector3.Dot(maxThrustForce, overallThrustDirection);
        }

        public Vector3 getDirection(int i) {
            return getOrientation(i) * Vector3.back;
        }

        public Vector3 getCurrentDirection(int i) {
            return (Quaternion.Inverse(vessel.transform.rotation) * thrustTransforms[i].rotation) * Vector3.back;
        }

        public Vector3 getCurrentPosition(int i) {
            return vessel.transform.InverseTransformPoint(thrustTransforms[i].position);
        }

        public Vector3 getPosition(int i) {
            if (initTransforms != null) {
                return vessel.transform.InverseTransformPoint(initTransforms[i].position);
            } else {
                return vessel.transform.InverseTransformPoint(thrustTransforms[i].position);
            }
        }

        public Quaternion getOrientation(int i) {
            if (initTransforms != null) {
                return Quaternion.Inverse(vessel.transform.rotation) * initTransforms[i].rotation;
            } else {
                return Quaternion.Inverse(vessel.transform.rotation) * thrustTransforms[i].rotation;
            }
        }

        private void updateThrustInformation() {
            Vector3d newMaxThrustForce = Vector3d.zero;
            Vector3d newMaxTorque = Vector3d.zero;

            double mxThrust = maxThrust / NumThrustTransforms;
            for (int i = 0; i < NumThrustTransforms; i++) {
                Vector3d direction = getDirection(i);
                Vector3d offset = getPosition(i) - vesselCoM;
                Vector3d maxForce = direction * mxThrust;

                newMaxThrustForce += maxForce;
                newMaxTorque += Vector3.Cross(offset, maxForce);
            }

            thrustDirection = newMaxThrustForce.normalized;
            maxThrustForce = newMaxThrustForce;
            maxTorque = newMaxTorque;
        }

    }
}

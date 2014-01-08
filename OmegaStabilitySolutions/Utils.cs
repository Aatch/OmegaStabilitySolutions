using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;

namespace OmegaSS {

    static class OSSDebug {
        public static void Log(string msg) {
#if DEBUG
            Debug.Log("[OSS] "+msg);
#endif
        }

        public static void Log(string format, params object[] o) {
            Log(String.Format(format, o));
        }

        public static void Error(string msg) {
            Debug.LogError("[OSS Error] " + msg);
        }

        public static void Error(string format, params object[] o) {
            Error(String.Format(format, o));
        }
    }

    struct MovingAverage {
        private double[] samples;
        private int curIndex;
        private int size;
        private int filled;

        public double Value {
            get {
                return getValue();
            }
            set {
                addValue(value);
            }
        }

        public MovingAverage(int size) {
            this.size = size;
            samples = new double[size];
            curIndex = 0;
            filled = 0;
        }

        public void addValue(double value) {
            samples[curIndex] = value;
            curIndex = (curIndex + 1) % size;
            if (filled < size) filled++;
        }

        public double getValue() {
            double val = 0;
            for (int i = 0; i < filled; i++) {
                val += samples[i];
            }
            return val / filled;
        }
    }

    public class PIDController {
        public double Kp, Ki, Kd;
        private double prevError = 0, intAccum = 0;

        public PIDController(double kp, double ki, double kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        public double Compute(double error) {
            intAccum += error * TimeWarp.fixedDeltaTime;
            double action = (Kp * error) + (Ki * intAccum) + (Kd * (error - prevError) / TimeWarp.fixedDeltaTime);
            prevError = error;
            return action;
        }

        public void Reset() {
            prevError = 0;
            intAccum = 0;
        }
    }

    public class PIDControllerV3 {
        public double Kp, Ki, Kd;
        private Vector3d prevError = Vector3d.zero, intAccum = Vector3d.zero;

        public PIDControllerV3(double kp, double ki, double kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        public Vector3d Compute(Vector3d error) {
            intAccum += error * TimeWarp.fixedDeltaTime;
            Vector3d action = (Kp * error) + (Ki * intAccum) + (Kd * (error - prevError) / TimeWarp.fixedDeltaTime);
            prevError = error;
            return action;
        }

        public void Reset() {
            prevError = Vector3.zero;
            intAccum = Vector3.zero;
        }
    }
}
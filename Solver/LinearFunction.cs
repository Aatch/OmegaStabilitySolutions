using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace OmegaSS.LP {
    public class LinearFunction {
        public Dictionary<Variable, double> Terms { get; private set;  }

        public LinearFunction() {
            Terms = new Dictionary<Variable, double>();
        }

        public double this[Variable v] {
            get {
                return getCoefficient(v);
            }
            set {
                setCoefficient(v, value);
            }
        }

        public double getCoefficient(Variable v) {
            double val;
            Terms.TryGetValue(v, out val);
            return val;
        }

        public void setCoefficient(Variable v, double value) {
            if (value == 0)
                Terms.Remove(v);
            else
                Terms.Add(v, value);
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using lpsolve55;

namespace OmegaSS.LP {
    public abstract class Variable {
        protected string name = null;

        public string Name {
            get { return name; }
            set {
                if (value == "") name = null;
                else name = value;
            }
        }

        public abstract void configureModel(int model, int varid);

        public virtual void dumpState() {
            OSSDebug.Log(Name);
        }
    }

    public class BoundedVariable : Variable {
        public double upperBound, lowerBound;
        public bool IsSemiCont { get; set; }

        public BoundedVariable() {
            this.upperBound = double.PositiveInfinity;
            this.lowerBound = 0;
            this.IsSemiCont = false;
        }

        public BoundedVariable(double lower, double upper, bool isSemiCont = false) {
            this.upperBound = upper;
            this.lowerBound = lower;
            this.IsSemiCont = isSemiCont;
        }

        public override void configureModel(int model, int index) {
            if (this.name != null) {
                lpsolve.set_col_name(model, index, this.name);
            }
            lpsolve.set_bounds(model, index, this.lowerBound,
                double.IsPositiveInfinity(this.upperBound) ? lpsolve.get_infinite(model) : this.upperBound);
            if (this.IsSemiCont)
                lpsolve.set_semicont(model, index, this.IsSemiCont);
        }

        public override void dumpState() {
            OSSDebug.Log("{0} <= {1} <= {2}", lowerBound, Name, upperBound);
        }
    }

    public class IntegerVariable : Variable {
        private int lower, upper;

        public IntegerVariable() {
            this.lower = 0;
            this.upper = int.MaxValue;
        }

        public IntegerVariable(int lower, int upper) {
            this.lower = lower;
            this.upper = upper;
        }

        public override void configureModel(int model, int index) {
            if (this.name != null) {
                lpsolve.set_col_name(model, index, this.name);
            }
            lpsolve.set_bounds(model, index, this.lower, this.upper);
            lpsolve.set_int(model, index, true);
        }
    }

    public class BinaryVariable : Variable {
        public override void configureModel(int model, int index) {
            if (this.name != null) {
                lpsolve.set_col_name(model, index, this.name);
            }
            lpsolve.set_binary(model, index, true);
        }
    }

    public class FreeVariable : Variable {
        public override void configureModel(int model, int index) {
            if (this.name != null) {
                lpsolve.set_col_name(model, index, this.name);
            }
            lpsolve.set_unbounded(model, index);
        }
    }
}

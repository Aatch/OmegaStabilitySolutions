using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace OmegaSS.LP {
    public class Problem {
        public LinearFunction Objective { get; set; }
        public List<Constraint> Constraints { get; private set; }

        public Problem() {
            Objective = new LinearFunction();
            Constraints = new List<Constraint>();
        }

        public void addConstraint(Constraint constraint) {
            Constraints.Add(constraint);
        }

        public void addConstraint(IEnumerable<Constraint> constraints) {
            Constraints.AddRange(constraints);
        }

        public bool removeConstraint(Constraint constraint) {
            return Constraints.Remove(constraint);
        }

        public bool removeConstraint(IEnumerable<Constraint> constraints) {
            bool removed = false;
            foreach (var c in constraints) {
                removed = removeConstraint(c) || removed;
            }
            return removed;
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using lpsolve55;
using UnityEngine;
using System.ComponentModel;

namespace OmegaSS.LP {

    public abstract class Solver : IDisposable {
        public enum Status {
            NotRun,
            Optimal,
            SubOptimal,
            Infeasible,
            Unbounded,
            Error
        }

        protected int lp_model = 0;

        private Problem problem = new Problem();
        public Problem Problem {
            get {
                return problem;
            }
        }
        protected Variable[] variables = null;

        public Variable[] Variables {
            get {
                return variables;
            } set {
                variables = value;
            }
        }

        private double[] results = null;
        private Status _status = Status.NotRun;
        public Status status {
            get {
                return _status;
            }
            protected set {
                _status = value;
            }
        }

        public double ObjectiveValue {
            get {
                if (lp_model == 0) return double.NaN;
                else return lpsolve.get_objective(lp_model);
            }
        }

        private Dictionary<LP.Variable, int> v_index = new Dictionary<Variable,int>();
        public double this[Variable v] {
            get {
                if (v_index.ContainsKey(v))
                    return results[v_index[v]];
                else
                    return 0;
            }
        }

        public Status solve() {
            if (variables == null) {
                HashSet<Variable> varSet = new HashSet<Variable>();
                foreach (Constraint c in Problem.Constraints) {
                    foreach (Variable v in c.fn.Terms.Keys) {
                        varSet.Add(v);
                    }
                }
                variables = varSet.ToArray();
            } else {
                variables = new HashSet<Variable>(variables).ToArray();
            }

            int nRows = Problem.Constraints.Count;
            int nCols = variables.Count();

            if (lp_model == 0) {
                lp_model = lpsolve.make_lp(nRows, nCols);
                if (lp_model == 0) {
                    throw new Exception("Failed to make lp model");
                }
            } else {
                lpsolve.resize_lp(lp_model, nRows, nCols);
            }

            double[] col = new double[nRows + 1];

            v_index.Clear();
            for (int i = 0; i < nCols; i++) {
                Variable v = variables[i];
                v_index[v] = i;
                v.configureModel(lp_model, i+1);
                col[0] = Problem.Objective[v];
                for (int j = 0; j < nRows; j++) {
                    col[j + 1] = Problem.Constraints[j][v];
                }
                lpsolve.set_column(lp_model, i + 1, col);
            }

            for (int i = 0; i < nRows; i++) {
                Constraint c = Problem.Constraints[i];

                lpsolve.set_constr_type(lp_model, i + 1, Constraint.convertType(c.type));
                lpsolve.set_rh(lp_model, i + 1, c.RHS);
            }

            this.configureSolver();

            lpsolve.lpsolve_return status = lpsolve.solve(lp_model);
            if (status == lpsolve.lpsolve_return.NUMFAILURE) {
                lpsolve.default_basis(lp_model);
                status = lpsolve.solve(lp_model);
            }
            this.status = convertLPStatus(status);

            results = new double[nCols];
            lpsolve.get_variables(lp_model, results);


            return this.status;
        }

        protected static Status convertLPStatus(lpsolve.lpsolve_return s) {
            switch (s) {
                case lpsolve.lpsolve_return.OPTIMAL:    return Status.Optimal;
                case lpsolve.lpsolve_return.SUBOPTIMAL: return Status.SubOptimal;
                case lpsolve.lpsolve_return.INFEASIBLE: return Status.Infeasible;
                case lpsolve.lpsolve_return.UNBOUNDED:  return Status.Unbounded;
                default:
                    if ((int)s == -1)
                        throw new Win32Exception();
                    return Status.Error;
            }
        }

        public void dumpState() {
            Variable[] modelVariables;
            if (variables == null) {
                HashSet<Variable> varSet = new HashSet<Variable>();
                foreach (Constraint c in Problem.Constraints) {
                    foreach (Variable v in c.fn.Terms.Keys) {
                        varSet.Add(v);
                    }
                }
                modelVariables = varSet.ToArray();
            } else {
                modelVariables = new HashSet<Variable>(variables).ToArray();
            }

            OSSDebug.Log("Variables:");
            foreach (var v in modelVariables) {
                v.dumpState();
            }
            OSSDebug.Log("Objective Function");
            string line = "    ";
            bool first = true;
            foreach (var v in modelVariables) {
                if (Problem.Objective[v] == 0) continue;
                if (first) {
                    line += String.Format("{0} * {1:F4}", v.Name, Problem.Objective[v]);
                    first = false;
                } else {
                    line += String.Format(" + {0} * {1:F4}", v.Name, Problem.Objective[v]);
                }
            }
            OSSDebug.Log(line);
            

            OSSDebug.Log("Constraints:");
            foreach (var constraint in Problem.Constraints) {
                line = "    ";
                first = true;
                foreach (var v in modelVariables) {
                    if (constraint[v] == 0) continue;
                    if (first) {
                        line += String.Format("{0} * {1:F4}", v.Name, constraint[v]);
                        first = false;
                    } else {
                        line += String.Format(" + {0} * {1:F4}", v.Name, constraint[v]);
                    }
                }
                if (first) continue;
                switch (constraint.type) {
                    case Constraint.Type.EQ:
                        line += " = ";
                        break;
                    case Constraint.Type.GE:
                        line += " >=";
                        break;
                    case Constraint.Type.LE:
                        line += " <=";
                        break;
                }
                line += String.Format(" {0:F4}", constraint.RHS);
                OSSDebug.Log(line);
            }

            if (status != Status.NotRun) {
                OSSDebug.Log("Results:");
                foreach (var v in modelVariables) {
                    OSSDebug.Log("{0} = {1:F4}", v.Name, this[v]);
                }
                OSSDebug.Log("Objective Value: {0:F4}", this.ObjectiveValue);
            }
        }

        protected abstract void configureSolver();

        public void finished() {
            int model = lp_model;
            lp_model = 0;
            if (model != 0) {
                lpsolve.delete_lp(model);
            }
        }

        void IDisposable.Dispose() {
            finished();
        }
    }

    public class MaxSolver : Solver {

        protected override void configureSolver() {
            lpsolve.set_maxim(lp_model);
        }

    }

    public class MinSolver : Solver {

        protected override void configureSolver() {
            lpsolve.set_minim(lp_model);
        }

    }
}

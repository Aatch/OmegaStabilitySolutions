using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using lpsolve55;
using UnityEngine;

namespace OmegaSS.LP {
    /// <summary>
    /// Represents a constraint for the solver.
    /// </summary>
    /// <remarks>
    /// A constraint is a row in the set of linear equations, of the form 'cx + dy + ez + ... = r',
    /// however the right hand side can be an inequality.
    /// </remarks>
    public class Constraint {
        /// <summary>
        /// Represents the type of the constraint
        /// </summary>
        public enum Type {
            /// <summary>
            /// Constrained to solutions that are less than or equal to the RHS
            /// </summary>
            LE, 
            /// <summary>
            /// Constrained to solutions that are equal to RHS
            /// </summary>
            EQ,
            /// <summary>
            /// Constrainted to solutions that are greater than or equal to the RHS
            /// </summary>
            GE
        }

        /// <summary>
        /// The linear function of this constraint. The left hand side of the equation.
        /// </summary>
        public LinearFunction fn { get; private set; }
        /// <summary>
        /// The right hand side of this constraint, defaults to 0;
        /// </summary>
        public double RHS { get; set; }
        /// <summary>
        /// The type of this constraint. Defaults to <c>Type.EQ</c>
        /// </summary>
        public Type type { get; set; }

        /// <summary>
        /// Constructs a new constraint with default values
        /// </summary>
        public Constraint() {
            fn = new LinearFunction();
            RHS = 0;
            type = Type.EQ;
        }

        /// <summary>
        /// Constructs a new constraint from existing values
        /// </summary>
        /// <param name="fn">The linear function to use. This is not copied, but stored as a reference.</param>
        /// <param name="rhs">The value for RHS</param>
        /// <param name="type">The type of the constraint</param>
        public Constraint(LinearFunction fn, double rhs = 0, Type type = Type.EQ) {
            this.fn = fn;
            this.RHS = rhs;
            this.type = type;
        }

        /// <summary>
        /// Gets the coefficient value from the constraint's linear function.
        /// </summary>
        /// <param name="v">The variable to get the coefficient of</param>
        /// <returns>The coefficient or zero if it isn't in the constraint</returns>
        /// <seealso cref="LinearFunction[Variable]"/>
        public double this[Variable v] {
            get {
                return fn[v];
            }
            set {
                fn[v] = value;
            }
        }

        public static lpsolve.lpsolve_constr_types convertType(Type ty) {
            switch (ty) {
                case Type.LE: return lpsolve.lpsolve_constr_types.LE;
                case Type.EQ: return lpsolve.lpsolve_constr_types.EQ;
                case Type.GE: return lpsolve.lpsolve_constr_types.GE;
                default:
                    throw new ArgumentOutOfRangeException("ty", ty, "Type is not valid");
            }
        }
    }

    /// <summary>
    /// A Helper class that wraps 2 constraints and allows them to be
    /// manipulated simultaneously with vectors.
    /// </summary>
    /// <remarks>This implements IEnumarable and so can be passed to <see cref="Problem.addConstraint"/></remarks>
    public class BiConstraint : IEnumerable<Constraint> {
        private Constraint[] constraints;

        #region Vector Component Properties
        /// <summary>
        /// The x component constraint
        /// </summary>
        public Constraint x {
            get {
                return constraints[0];
            }
            set {
                constraints[0] = value;
            }
        }

        /// <summary>
        /// The y component constraint
        /// </summary>
        public Constraint y {
            get {
                return constraints[1];
            }
            set {
                constraints[1] = value;
            }
        }
        #endregion

        public BiConstraint() {
            constraints = new[] {
                new Constraint(), new Constraint(), new Constraint()
            };
        }

        public Constraint this[int i] {
            get {
                return constraints[i];
            }
            set {
                if (i >= 0 && i <= 1)
                    constraints[i] = value;
            }
        }

        public Vector2d this[Variable v] {
            get {
                return new Vector2d(this.x[v], this.y[v]);
            }
            set {
                this.x[v] = value.x;
                this.y[v] = value.y;
            }
        }

        public Vector2d RHS {
            get {
                return new Vector2d(this.x.RHS, this.y.RHS);
            }
            set {
                this.x.RHS = value.x;
                this.y.RHS = value.y;
            }
        }

        #region IEnumberable implementation
        public IEnumerator<Constraint> GetEnumerator() {
            return constraints.AsEnumerable().GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return constraints.GetEnumerator();
        }
        #endregion
    }

    /// <summary>
    /// A Helper class that wraps 3 constraints and allows them to be
    /// manipulated simultaneously with vectors.
    /// </summary>
    /// <remarks>This implements IEnumarable and so can be passed to <see cref="Problem.addConstraint"/></remarks>
    public class TriConstraint : IEnumerable<Constraint> {

        private Constraint[] constraints;

        #region Vector Component Properties
        /// <summary>
        /// The x component constraint
        /// </summary>
        public Constraint x {
            get {
                return constraints[0];
            }
            set {
                constraints[0] = value;
            }
        }

        /// <summary>
        /// The y component constraint
        /// </summary>
        public Constraint y {
            get {
                return constraints[1];
            }
            set {
                constraints[1] = value;
            }
        }

        /// <summary>
        /// The z component constraint
        /// </summary>
        public Constraint z {
            get {
                return constraints[2];
            }
            set {
                constraints[2] = value;
            }
        }
        #endregion

        public TriConstraint() {
            constraints = new[]{
                new Constraint(), new Constraint(), new Constraint()
            };
        }

        /// <summary>
        /// Gets and sets the constraint by numerical index, 0 = x, 1 = y, 2 = z
        /// </summary>
        /// <remarks>If the index is out of bounds, then does nothing</remarks>
        public Constraint this[int i] {
            get {
                return constraints[i];
            }
            set {
                if (i >= 0 && i <= 2)
                    constraints[i] = value;
            }
        }

        /// <summary>
        /// Gets and sets the coefficients for the given variable as a <see cref="Vector3d"/>
        /// <seealso cref="Constraint[Variable v]"/>
        /// </summary>
        public Vector3d this[Variable v] {
            get {
                return new Vector3d(
                    this.x[v],
                    this.y[v],
                    this.z[v]);
            }
            set {
                this.x[v] = value.x;
                this.y[v] = value.y;
                this.z[v] = value.z;
            }
        }

        /// <summary>
        /// The right hand side of the component constraints as a <see cref="Vector3d"/>
        /// </summary>
        public Vector3d RHS {
            get {
                return new Vector3d(
                    this.x.RHS,
                    this.y.RHS,
                    this.z.RHS);
            }
            set {
                this.x.RHS = value.x;
                this.y.RHS = value.y;
                this.z.RHS = value.z;
            }
        }

        #region IEnumerable Implementation
        public IEnumerator<Constraint> GetEnumerator() {
            return constraints.AsEnumerable().GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return constraints.GetEnumerator();
        }
        #endregion

        public static TriConstraint Clone(TriConstraint src) {
            var c = new TriConstraint();
            c.x = new Constraint(src.x.fn);
            c.y = new Constraint(src.y.fn);
            c.z = new Constraint(src.z.fn);
            return c;
        }
    }

    abstract class ConstraintSet : IEnumerable<Constraint> {
        protected abstract IEnumerable<Constraint> GetConstraints();

        #region IEnumberable Implementation
        public IEnumerator<Constraint> GetEnumerator() {
            return GetConstraints().GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() {
            return GetConstraints().GetEnumerator();
        }
        #endregion
    }
}

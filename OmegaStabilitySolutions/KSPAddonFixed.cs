using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace OmegaSS {
    public class KSPAddonFixed : KSPAddon, IEquatable<KSPAddonFixed> {

        private readonly Type type;

        public KSPAddonFixed(KSPAddon.Startup startup, bool once, Type type)
            : base(startup, once) {

            this.type = type;
        }

        public override bool Equals(object other) {
            if (other.GetType() != this.GetType()) return false;
            return Equals((KSPAddonFixed)other);
        }

        public bool Equals(KSPAddonFixed other) {
            return
                this.once == other.once &&
                this.startup == other.startup &&
                this.type == other.type;
        }

        public override int GetHashCode() {
            return this.startup.GetHashCode() ^ this.once.GetHashCode() ^ this.type.GetHashCode();
        }
    }
}

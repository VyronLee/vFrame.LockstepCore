namespace vFrame.Lockstep.Core {

    /**
    * @brief Represents an interface to 2D bodies.
    **/
    public interface IBody2D : IBody {

        /**
        * @brief If true the body doesn't move around by collisions.
        **/
        bool TSIsStatic {
            get; set;
        }
        
        /**
         *  @brief Static friction when in contact. 
         **/
        FP TSFriction {
            get; set;
        }

        /**
        * @brief Coeficient of restitution.
        **/
        FP TSRestitution {
            get; set;
        }

        /**
        * @brief Set/get body's position.
        **/
        TSVector2 TSPosition {
            get; set;
        }

    }

}
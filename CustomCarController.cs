using UnityEngine;

public class CustomCarController : MonoBehaviour
{
    [Header("Car params")]

    [SerializeField] Vector3 centerOfMassOffset;

    [Header("Suspension")]

    [SerializeField] float restLength;
    [SerializeField] float suspension_strength;
    [SerializeField] float suspension_damping;

    [Header("Steering")]

    [SerializeField] private float steerTime = 8;

    [SerializeField] AnimationCurve frontTiresGrip; 
    [SerializeField] AnimationCurve backTiresGrip; 

    //[SerializeField]float tireGripFactor;

    [Tooltip("In meters")]
    public float wheelBase;
    [Tooltip("In meters")]
    public float rearTrack;
    [Tooltip("In meters")]
    public float turnRadius;


    [Header("Acceleration")]

    [SerializeField] float carTopSpeed;
    [SerializeField] float horsePower;
    [SerializeField] float velocityLoss;
    [SerializeField] AnimationCurve powerCurve;

    [SerializeField] float brakeFactor;

    [Header("Wheels")]
    [SerializeField] float wheelRadius;

    //Inputs-----------

    float steerInput;
    float accelerationInput;

    float ackermannAngleLeft;
    float ackermannAngleRight;

    float maxLength;

    Rigidbody rb;

    [System.Serializable]
    struct Wheel {
        //ONLY FRONT wheels need A parent trasnform
        public Transform parentTransform;        
        
        public Transform transform;
        public Transform wheelGfx;
        public bool driverWheel;

        [HideInInspector]
        public float steerAngle;
        [HideInInspector]
        public float startRotationY;
        [HideInInspector]
        public float wheelAngle;

        [HideInInspector]
        public float currentDeceleration;
    }

    [SerializeField] Wheel wheelFR;
    [SerializeField] Wheel wheelFL;
    [SerializeField] Wheel wheelBR;
    [SerializeField] Wheel wheelBL;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = rb.centerOfMass + centerOfMassOffset;

        maxLength = restLength + wheelRadius;

        wheelFR.startRotationY = wheelFR.transform.localEulerAngles.y;
        wheelBR.startRotationY = wheelBR.transform.localEulerAngles.y;
        wheelFL.startRotationY = wheelFL.transform.localEulerAngles.y;
        wheelBL.startRotationY = wheelBL.transform.localEulerAngles.y;
    }

    private void Update()
    {
        steerInput = Input.GetAxis("Horizontal");
        accelerationInput = Input.GetAxis("Vertical");

        if (steerInput > 0)
        {
            ackermannAngleLeft = Mathf.Rad2Deg * Mathf.Atan(wheelBase/ (turnRadius + (rearTrack / 2))) * steerInput;
            ackermannAngleRight = Mathf.Rad2Deg * Mathf.Atan(wheelBase/ (turnRadius - (rearTrack / 2))) * steerInput;
        }
        else if (steerInput < 0)
        {
            ackermannAngleLeft = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (rearTrack / 2))) * steerInput;
            ackermannAngleRight = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (rearTrack / 2))) * steerInput;
        }
        else {
            ackermannAngleLeft = 0;
            ackermannAngleRight = 0;
        }

        wheelFR.steerAngle = ackermannAngleLeft * 2f;
        wheelFL.steerAngle = ackermannAngleRight * 2f;

        HandleWheelRotations(ref wheelFR);
        HandleWheelRotations(ref wheelFL);

        HandleWheelRunningRotation(ref wheelFR);
        HandleWheelRunningRotation(ref wheelFL);
        HandleWheelRunningRotation(ref wheelBR);
        HandleWheelRunningRotation(ref wheelBL);

    }
    void FixedUpdate()
    {
        HandleWheelPhysics(ref wheelFR, frontTiresGrip);
        HandleWheelPhysics(ref wheelFL, frontTiresGrip);
        HandleWheelPhysics(ref wheelBR, backTiresGrip);
        HandleWheelPhysics(ref wheelBL, backTiresGrip);
    }


    void HandleWheelPhysics(ref Wheel wheel, AnimationCurve tireGripCurve) {
        bool rayDidHit = Physics.Raycast(wheel.transform.position, -wheel.transform.up, out RaycastHit hit, maxLength);

        //suspension
        if (rayDidHit)
        {
            //Handle equlibrium case

            Vector3 springDir = wheel.transform.up;

            Vector3 tireWorldVel = rb.GetPointVelocity(wheel.transform.position);

            float offset = restLength - (hit.distance - wheelRadius);

            float springForce = offset * suspension_strength * rb.mass;

            float vel = Vector3.Dot(springDir, tireWorldVel);

            float dampingForce = vel * suspension_damping * rb.mass;

            float suspensionForce = springForce - dampingForce;

            rb.AddForceAtPosition(springDir * suspensionForce, wheel.transform.position);

            // Calculate the position where the wheel graphics should be
            Vector3 wheelGraphicsPosition = hit.point + wheel.transform.up * wheelRadius;


            if (wheel.parentTransform != null)
            {
                // Apply the calculated position to the wheel graphics
                wheel.parentTransform.position = wheelGraphicsPosition;
            }
            else { 
                wheel.wheelGfx.position = wheelGraphicsPosition;
            }
        }

        //steering
        if (rayDidHit)
        {
            
            Vector3 slippingDir = wheel.transform.right;

            Vector3 velocityAtPoint = rb.GetPointVelocity(wheel.transform.position);

            float steeringVel = Vector3.Dot(slippingDir, velocityAtPoint);

            float desiredVelChange = -steeringVel * tireGripCurve.Evaluate(steeringVel);

            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

            rb.AddForceAtPosition(slippingDir * rb.mass/4 * desiredAccel, wheel.transform.position);

            /*
            Vector3 velocityAtPoint = rb.GetPointVelocity(wheel.transform.position);

            // Calculate the force needed to counteract the velocity at that point
            Vector3 forceToStop = -velocityAtPoint * rb.mass;

            // Apply an equal and opposite force to counteract the velocity at the stop point
            rb.AddForceAtPosition(forceToStop * tireGripFactor, wheel.transform.position);
            */
        }

        //acceleration
        if (rayDidHit)
        {
            if (!wheel.driverWheel) return;
            Vector3 accelDir = wheel.transform.forward;

            if (Mathf.Abs(accelerationInput) > 0.0f)
            {
                float carSpeed = Vector3.Dot(transform.forward, rb.velocity);

                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / (carTopSpeed * rb.mass / 3.6f));

                float availableTorque = powerCurve.Evaluate(normalizedSpeed) * accelerationInput;

                if (Mathf.Sign(accelerationInput) != Mathf.Sign(carSpeed))
                {
                    rb.AddForceAtPosition(accelDir * availableTorque * horsePower * rb.mass/4 * 5, wheel.transform.position);
                }
                else { 
                    rb.AddForceAtPosition(accelDir * availableTorque * horsePower * rb.mass / 4, wheel.transform.position);
                }
                wheel.currentDeceleration = 0;
            }
            else
            {
                wheel.currentDeceleration = Mathf.Lerp(wheel.currentDeceleration,velocityLoss, 0.05f);
      
                Vector3 wheelVel = rb.GetPointVelocity(wheel.transform.position);
                float carSpeed = Vector3.Dot(wheel.transform.forward, wheelVel);
                if (Mathf.Abs(carSpeed) > 0.005f)
                {
                    rb.AddForceAtPosition(-accelDir.normalized * wheel.currentDeceleration * rb.mass/4 * Mathf.Sign(carSpeed), wheel.transform.position);
                }
                else if (Mathf.Abs(carSpeed) > 0.0005f)
                {
                    Vector3 localVelocity = wheel.transform.InverseTransformDirection(wheelVel);
                    localVelocity.y = 0;

                    Vector3 globalVelocity = wheel.transform.TransformDirection(localVelocity);

                    rb.AddForceAtPosition(-globalVelocity * rb.mass/4, wheel.transform.position);
                }
            }
        }

    }

    void HandleWheelRotations(ref Wheel wheel)
    {
        wheel.wheelAngle = Mathf.Lerp(wheel.wheelAngle, wheel.steerAngle, 0.1f);

        // Calculate the new Y-axis rotation for the wheel's visual representation
        float newRotationY = wheel.startRotationY + wheel.wheelAngle;

        // Ensure the Y-axis rotation stays within the 0 to 360 degrees range
        newRotationY = Mathf.Repeat(newRotationY, 360f);

        // Get the current local Euler angles of the wheelGfx
        Vector3 currentGfxEulerAngles = wheel.parentTransform.localEulerAngles;

        // Calculate the new Euler angles for the wheelGfx, preserving the X-axis rotation
        Vector3 newGfxEulerAngles = new Vector3(currentGfxEulerAngles.x, newRotationY, currentGfxEulerAngles.z);

        // Convert the new Euler angles to a quaternion
        Quaternion newGfxRotation = Quaternion.Euler(newGfxEulerAngles);

        // Apply the new rotation to the wheelGfx
        wheel.transform.localRotation = newGfxRotation;
        wheel.parentTransform.localRotation = newGfxRotation;
    }


    void HandleWheelRunningRotation(ref Wheel wheel) {

        Vector3 wheelVelocity = rb.GetPointVelocity(wheel.transform.position);

        // Calculate the speed of the wheel along its local forward axis
        float forwardSpeed = Vector3.Dot(wheelVelocity, wheel.transform.forward);
        if (Mathf.Abs(forwardSpeed) <= 0.1f) return;

        // Define the amount of rotation you want to add to the X axis
        float rotateAmountX = forwardSpeed; // Change this value as needed

        // Create a quaternion for rotation around the object's local X-axis
        Quaternion xRotation = Quaternion.AngleAxis(rotateAmountX, Vector3.right);

        // Apply the rotation to the object's local rotation
        wheel.wheelGfx.localRotation *= xRotation;
    }
}

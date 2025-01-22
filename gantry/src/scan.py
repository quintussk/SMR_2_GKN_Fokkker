import asyncio
from arduino import ArduinoConnection
from camera import Camera

class Scanning:
    def __init__(self, arduinoClass: ArduinoConnection, camera: Camera):
    # def __init__(self):

        # Define the movement in centimeters for X and Y directions
        self.X_Movement = 30  # How many cm to move for the X position (Gantry)
        self.Y_Movement = 28  # How many cm to move for the Y position (Camera)
        
        self.Max_Camera_Movement = 149  # Maximum movement for the camera in cm

        # Define the hardware-specific stepper motor details
        self.Stepper_Rev_Ratio = 400  # Steps per full revolution of the motor

        self.Wheel_Circumference = 18.1 # Circumference of the wheel in cm
        self.Camera_Ration = 15.5 # Ratio of the camera movement to the mold movement Rev/CM

        self.Steps_per_cm_wheel = int(self.Stepper_Rev_Ratio / self.Wheel_Circumference)
        self.Steps_per_cm_camera = int(self.Stepper_Rev_Ratio / self.Camera_Ration)

        # Calculate steps required to move 1 cm
        self.Steps_per_cm = int(self.Stepper_Rev_Ratio / self.Wheel_Circumference)

        # Started Arduino class
        self.arduinoClass = arduinoClass

        # Started Camera class
        self.camera = camera

        # Initialize current positions
        self.current_X = 0  # Current X position in cm
        self.current_Y = 0  # Current Y position in cm

    async def check_if_camera_is_home(self):
        """
        Checks if the camera is at the home position.
        """
        position_reached = await self.arduinoClass.check_camera()
        if not position_reached:
            await self.arduinoClass.change_speed_motor("Camera",100)
            await self.arduinoClass.home_camera()
        print("Camera is homed")
        

    async def Start_Scanning(self, X_Total: int, Y_Total: int, mold: str):
        """
        Main scanning function.
        - Calculates the movement plan based on total X and Y dimensions.
        - Executes the movement plan step by step.
        """

        await self.check_if_camera_is_home()
        # Ensure the inputs are integers
        await self.arduinoClass.change_speed_motor(motor="Mold", speed=500)
        await self.arduinoClass.change_speed_motor(motor="Camera", speed=1000)
        try:
            X_Total = int(X_Total)
            Y_Total = int(Y_Total)
        except ValueError:
            raise ValueError("X_Total and Y_Total must be integers.")
        
        # Calculate the movement plan for the given X_Total and Y_Total dimensions
        movement_plan = await self.Calculate_Movement(X_Total, Y_Total, "Negative")

        # Execute each movement in the calculated plan
        for step in movement_plan:
            if step["axis"] == "Y":
                # Move the camera along the Y-axis
                await self.Move_Camera(step["steps"])
            elif step["axis"] == "X":
                # Move the gantry along the X-axis
                await self.Move_Gantry(step["steps"])
            # Take a photo at the current location
            await self.camera.take_picture(mold_name=mold, filename=f"{mold}_image_X{self.current_X}_Y{self.current_Y}.jpg")
            print(f"Photo taken at X: {self.current_X} cm, Y: {self.current_Y} cm")

    async def Calculate_Movement(self, X_Total: int, Y_Total: int, Direction: str):
            """
            Calculates the movement steps required for scanning without overshooting.
            Ensures that the total movement along the Y-axis does not exceed the maximum camera movement.
            Args:
               X_Total (int): The total distance to move along the X-axis in centimeters.
                Y_Total (int): The total distance to move along the Y-axis in centimeters.
                Direction (str): The direction of movement ("Positive" or "Negative").
            Returns:
                movement_plan (list): A list of dictionaries describing each movement step.
            """
            movement_plan = []  # List to store the movement plan

            # Adjust Y_Total to fit within the maximum allowed movement
            if Y_Total > self.Max_Camera_Movement:
                print(f"Requested Y_Total ({Y_Total} cm) exceeds Max_Camera_Movement ({self.Max_Camera_Movement} cm). Adjusting.")
                Y_Total = self.Max_Camera_Movement

            # Calculate the total allowed steps for X and Y dimensions
            total_X_Steps = int(X_Total * self.Steps_per_cm_wheel)  # Total steps for the X dimension
            total_Y_Steps = int(Y_Total * self.Steps_per_cm_camera)  # Total steps for the adjusted Y dimension

            print(f"Total X Steps: {total_X_Steps}, Total Y Steps: {total_Y_Steps}")

            # Calculate the initial half-step in the X direction
            half_X_Steps = int((self.X_Movement / 2) * self.Steps_per_cm_wheel)

            # Adjust the initial half-step to ensure it does not cause an overshoot
            if half_X_Steps + int((X_Total // self.X_Movement) * self.X_Movement * self.Steps_per_cm_wheel) > total_X_Steps:
                # Reduce the initial half-step to prevent overshooting
                half_X_Steps = total_X_Steps % int(self.X_Movement * self.Steps_per_cm_wheel)

            # Add the initial half-step to the movement plan
            movement_plan.append({"axis": "X", "steps": half_X_Steps})

            # Calculate the steps for each full movement in X and Y directions
            X_Steps = int(self.X_Movement * self.Steps_per_cm_wheel)
            Y_Steps = int(self.Y_Movement * self.Steps_per_cm_camera)

            # Determine how many full movements fit in the total dimensions
            x_moves = X_Total // self.X_Movement  # Number of full movements in the X direction
            y_moves = Y_Total // self.Y_Movement  # Number of full movements in the Y direction

            X_Direction = 1 if Direction == "Positive" else -1
            # Loop through each X movement (columns)
            for x in range(x_moves):
                # Determine the Y direction for the current column
                Y_Direction = -1 if x % 2 == 0 else 1  # Start with negative (-1) for even columns, positive (1) for odd columns

                # Loop through each Y movement (rows) in the current column
                for _ in range(y_moves):
                    # Add a movement in the Y direction to the plan
                    movement_plan.append({"axis": "Y", "steps": Y_Steps * Y_Direction})

                # Add a movement in the X direction to the plan, unless it's the last column
                if x < x_moves - 1:
                    movement_plan.append({"axis": "X", "steps": X_Steps * X_Direction})

            
            print(movement_plan)
            return movement_plan  # Return the complete movement plan

    async def Move_Gantry(self, steps: int):
        """
        Movement of the gantry along the X-axis in steps.
        """
        self.arduinoClass.send_steps(steps, 0)  # Send inverted steps to Arduino
        await self.arduinoClass.Wait_For_Location_Reached()
        # Update current X position
        self.current_X += int(round(steps / self.Steps_per_cm))
        print(f"Gantry has reached the location at X: {self.current_X} cm")

    async def Move_Camera(self, steps: int):
        """
        Movement of the camera along the Y-axis in steps.
        """
        self.arduinoClass.send_steps(0,steps)  # Send steps to Arduino
        await self.arduinoClass.Wait_For_Location_Reached()
        # Update current Y position
        self.current_Y += int(round(steps / self.Steps_per_cm))
        print(f"Camera has reached the location at Y: {self.current_Y} cm")


if __name__ == "__main__":
    # Create a Scanning instance
    # arduino = ArduinoConnection(port="/dev/cu.usbmodem11301")
    # camera = Camera()
    scan = Scanning()
    # Start scanning with given X_Total and Y_Total dimensions
    asyncio.run(scan.Calculate_Movement(60,215,"Negative"))


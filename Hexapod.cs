#if DEBUG
using System;
using System.Linq;
using System.Text;
using System.Collections;
using System.Collections.Generic;

using VRageMath;
using VRage.Game;
using VRage.Collections;
using Sandbox.ModAPI.Ingame;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using Sandbox.Game.EntityComponents;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;

namespace Hexapod
{
	public sealed class Program : MyGridProgram
	{
#endif
		HexapodWorker worker;
		Program()
		{
			try
			{
				worker = new HexapodWorker(GridTerminalSystem, this.Echo);
			}
			catch (Exception e)
			{
				Echo(e.Message);
				Echo(e.Source);
				Echo(e.StackTrace);
				throw;
			}
			Runtime.UpdateFrequency = UpdateFrequency.Update10;
		}

		void Main(string argument, UpdateType updateType)
		{
			Echo("Tick " + updateType);
			Echo("State  " + worker.currentState);
			Echo("Legs " + worker.legController?.Legs?.Count);
			try
			{
				worker.Tick(argument);
			}
			catch (Exception e)
			{
				Echo(e.Message);
				Echo(e.Source);
				Echo(e.StackTrace);
			}
		}

		public class HexapodWorker
		{
			public static bool DEBUG = false;
			IMyGridTerminalSystem gridTerminalSystem;
			List<IMyShipController> controllers = new List<IMyShipController>();

			IMyShipController GetCurrentController()
			{
				return controllers.FirstOrDefault(x => x.CanControlShip && (x.IsUnderControl || ((x as IMyRemoteControl)?.IsAutoPilotEnabled ?? false)));
			}

			IMyShipController currentController;

			List<string> debugMessages = new List<string>();
			IDictionary<string, Action> arguments;

			public LegController legController;

			Action<string> Echo;

			public HexapodWorker(IMyGridTerminalSystem gridTerminalSystem, Action<string> echo)
			{
				this.gridTerminalSystem = gridTerminalSystem;
				this.Echo = echo;

				arguments = new Dictionary<string, Action> {
					{"", () => { } },
					{"toggleMoveForward", () => { legController.moveForward = !legController.moveForward; } },
					{"speed+", () => { legController.maxTickStep = Math.Min(legController.maxTickStep+5,45); } },
					{"speed-", () => { legController.maxTickStep = Math.Max(legController.maxTickStep-5,5); } },
					{"toggleJump", () => { legController.jump = !legController.jump; } },
					{"toggleStrafe", () => { legController.strafe = !legController.strafe; } },
					{"toggleSlide", () => { legController.slide = !legController.slide; } },
				};
			}

			public enum State
			{
				WaitForPlayer,
				Setup,
				Walking,
				Sliding,
				Idle,
				Error
			}

			public State currentState = State.WaitForPlayer;

			public void Tick(string argument)
			{
				if (!string.IsNullOrWhiteSpace(argument))
				{
					arguments[argument]();
					return;
				}

				debugMessages.ForEach(Echo);

				switch (currentState)
				{
					case State.WaitForPlayer:
						WaitForPlayer();
						break;
					case State.Setup:
						Setup();
						break;
					case State.Walking:
						Walking();
						break;
					case State.Sliding:
						Sliding();
						break;
					case State.Idle:
						Idle();
						break;
				}
			}

			void WaitForPlayer()
			{
				gridTerminalSystem.GetBlocksOfType(controllers, x => x.CanControlShip);
				currentController = GetCurrentController();
				if (currentController == null)
				{
					return;
				}
				currentState = State.Setup;
			}
			void Setup()
			{
				try
				{
					gridTerminalSystem.GetBlocksOfType(controllers, x => x.CanControlShip);
					currentController = GetCurrentController();
					if (currentController == null)
					{
						return;
					}
					var legFactory = new LegFactory("InnerJointRotors", currentController, gridTerminalSystem, Echo, debugMessages, 45);
					legController = legFactory.GetLegs();

					this.currentState = State.Walking;
				}
				catch (Exception e)//setup failed, nothing to do in this case
				{
					this.currentState = State.Error;
					debugMessages.Add(e.Message);
					debugMessages.Add(e.Source);
					debugMessages.Add(e.StackTrace);
					throw;
				}

			}

			void Walking()
			{
				var newController = GetCurrentController();
				if (newController == null)
				{
					this.currentState = State.Idle;
					return;
				}

				var rc = currentController as IMyRemoteControl;
				if (rc != null)
					legController.UpdateFrom(rc);
				else
					legController.UpdateFrom(currentController);

				legController.Apply();
			}

			void Sliding()
			{
			}
			void Idle()
			{
				Echo($"Controllers: {controllers.Count}");
				currentController = GetCurrentController();
				Echo($"CurrentController: {currentController?.DisplayName}");
				if (currentController != null)
				{
					this.currentState = State.Setup;
				}
			}

			public class LegController
			{
				public int ticker = 0;
				public int tickStep = 5;
				public int maxTickStep = 45;

				//settings
				public bool moveForward = false;
				public bool jump = false;
				public bool strafe = true;
				public bool slide = false;

				//limits
				public float minHeight = 0;
				public float maxHeight = 0;
				public float defaultHeight = 0;
				public float maxPitch = (float)Math.PI / 4f;// 45degrees
				public float maxYaw = (float)Math.PI / 4f;// 45degrees

				//values
				public float stepHeightMult = 1;
				public float turnAngle = 0;
				public float previousStepAngle = 0;
				public float heightMod = 0;
				public float pitch = 0;//for mouse control
				public float yaw = 0;//for mouse control
				public float gravPitch = 0;//pitch to gravity vector
				public float gravRoll = 0;//roll to gravity vector
				public float accelPitch = 0;//
				public float accelRoll = 0;//used to tilt on acceleration

				public float vehicleSize = 1;//do things slower for large vehicles

				public double elevation = 0;//current elevation - used to disable actions while in air
				public Vector2 prevMoveVector = new Vector2();//in previous tick
				public Vector2 moveVector = new Vector2();//X - forward, Y - side

				public float globalXmod = 0;
				public float globalYmod = 0;
				public float globalZmod = 0;

				//leg control parameters
				public float moveAngle = 0;
				public float moveLenght = 0;
				public float landMult = .5f;

				public List<Leg> Legs { get; set; }
				public List<IMyGyro> Gyros { get; set; }
				public Action<string> Echo;

				float avgVelocity = 0;//average velocity for N last ticks
				DateTime lastTickTime = DateTime.UtcNow;
				TimeSpan tickTime;


				public LegController(IEnumerable<Leg> legs, Action<string> echo, IEnumerable<IMyGyro> gyros)
				{
					this.Legs = legs.OrderBy(x => Math.Atan2(x.Position.X, x.Position.Y)).ToList();
					this.Gyros = gyros.ToList();
					this.Echo = echo;


					var legCount = Legs.Count;
					var shiftIncrement = 180 + (int)(legCount / 60f * 180f);
					var phaseShift = 0;
					Legs.ForEach(x =>
					{
						phaseShift = (phaseShift + shiftIncrement) % 360;
						x.UpdateStep(phaseShift: phaseShift);
					});

					minHeight = Legs.Max(x => x.MidJoint.Length);
					maxHeight = -Legs.Max(x => x.MidJoint.Length + x.OuterJoint.Length) * .9f;
					defaultHeight = maxHeight / 2;
					vehicleSize = Legs.Max(x => x.Position.X) - Legs.Min(x => x.Position.X);

					var maxZ = Legs.Min(x => x.GetLegPosition().Z);
					heightMod = Math.Max(Math.Min(minHeight, maxZ), maxHeight);

					Legs.ForEach(x =>
					{
						x.UpdateStep(idleZ: heightMod);
					});
				}

				public void UpdateFrom(IMyShipController controller)
				{
					ticker = (ticker + tickStep) % 360;
					var now = DateTime.UtcNow;
					tickTime = now - lastTickTime;
					lastTickTime = now;


					if (controller.TryGetPlanetElevation(MyPlanetElevation.Surface, out elevation) && elevation > -maxHeight * 2)
					{//do nothing in air
						moveLenght = moveAngle = moveAngle = turnAngle = pitch = yaw = 0;
						return;
					}

					var gravityVec = controller.GetNaturalGravity();
					var gravityVecLength = gravityVec.Length();

					var referenceForward = controller.WorldMatrix.Forward;
					var referenceLeft = controller.WorldMatrix.Left;
					gravPitch = (float)Math.Atan(gravityVec.Dot(referenceForward) / gravityVecLength);
					gravRoll = (float)Math.Atan(gravityVec.Dot(referenceLeft) / gravityVecLength);

					var inputVec = controller.MoveIndicator;

					prevMoveVector.X = moveVector.X;
					prevMoveVector.Y = moveVector.Y;

					//Steering 
					moveVector.X -= inputVec.Z / 5f;
					if (moveForward) moveVector.X += 10;
					if (strafe) moveVector.Y += inputVec.X / 5f;


					//Turn 
					if (!strafe)
					{
						turnAngle = (float)(Math.Sign(inputVec.X) * Math.PI / 6f);
					}
					else
					{
						turnAngle = 0;
					}

					//stand up / sit down
					if (!jump)
					{
						heightMod -= Math.Sign(inputVec.Y) * .5f;
						heightMod = Math.Max(Math.Min(heightMod, minHeight), maxHeight);
					}
					//jump

					if (jump && inputVec.Y > 0)
					{
						globalZmod = globalXmod = (maxHeight - minHeight) / 2;
					}
					else
					{
						globalZmod = globalXmod = 0;
					}

					moveVector *= .9f;//slowdown if no input
					if (moveVector.Length() > 1)
					{
						moveVector.Normalize();
					}
					if (moveVector.Length() < .1)
					{
						moveVector.X = 0;
						moveVector.Y = 0;
					}

					accelPitch = (float)Math.Atan2(moveVector.X - prevMoveVector.X, .2);
					accelRoll = (float)Math.Atan2(moveVector.Y - prevMoveVector.Y, .2);

					moveAngle = (float)Math.Atan2(moveVector.Y, moveVector.X);
					moveLenght = moveVector.Length();
					moveLenght -= moveLenght * Math.Abs(moveVector.Y) * .2f;//decrease move length for strafe

					pitch += controller.RotationIndicator.X / 100 / vehicleSize;
					pitch = Math.Min(Math.Max(-maxPitch, pitch), maxPitch);

					yaw += controller.RotationIndicator.Y / 100 / vehicleSize;
					yaw *= 0.99f;
					if (Math.Abs(yaw) > maxYaw)//step for turn
					{
						turnAngle += MathHelper.ToDegrees(yaw);
						yaw = Math.Min(Math.Max(-maxYaw * 1.2f, yaw), maxYaw * 1.2f);
					}
					if (Math.Abs(turnAngle) < turnAngle / 10)
					{
						yaw = 0;
					}

					var controllerMatrix = controller.WorldMatrix;

					var velocityVector = controller.GetShipVelocities().LinearVelocity;
					var angularVelocity = Vector3D.TransformNormal(controller.GetShipVelocities().AngularVelocity, controllerMatrix);
					var velocityForward = velocityVector.Dot(controllerMatrix.Forward);
					var velocityRigth = velocityVector.Dot(controllerMatrix.Right);
					var alignedVelocity = new Vector2D(velocityForward, velocityRigth);
					Echo($"AV {angularVelocity}");
					Echo($"VV {alignedVelocity}");
					var velocity = (float)alignedVelocity.Length();
					var expectedVelocity = Legs.Select(x => x.GetEstimatedSpeed(tickStep, tickTime)).Average();

					var expectedVelocityVector = new Vector2D(expectedVelocity * Math.Cos(moveAngle), expectedVelocity * Math.Sin(moveAngle));
					Echo($"EV {expectedVelocityVector}");

					Echo($"Move angle {moveAngle}");
					if (moveVector.Length() == 0 && angularVelocity.Length() > .1 && (alignedVelocity - expectedVelocityVector).Length() > 3d)//no user input and walker move in some direction
					{//walk on move direction to keep balance
						moveAngle = (float)Math.Atan2(alignedVelocity.Y, alignedVelocity.X);
						moveLenght = 1;
						avgVelocity = velocity * .9f;
						Echo($"Move angle {moveAngle}");
					}

					avgVelocity += (velocity - avgVelocity) / 10f;
					Echo($"Move forward {moveForward}");
					Echo($"Actual velocity     {Math.Round(avgVelocity, 4)}\nExpected velocity {Math.Round(expectedVelocity, 4)}");

					if (avgVelocity > .5 && avgVelocity > expectedVelocity * .8)
					{
						tickStep = Math.Min(tickStep + 5, maxTickStep);
					}
					else if (avgVelocity < .5 || avgVelocity < expectedVelocity * 2)
					{
						tickStep = Math.Max(tickStep - 5, 5);
					}

					landMult = .7f - Math.Min(tickStep / 50, .2f);

					/////////////////////////
					var rotationVec = new Vector3D(-gravPitch, 0, gravRoll);
					var shipMatrix = controller.WorldMatrix;
					var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);
					foreach (var gyro in Gyros)
					{
						var gyroMatrix = gyro.WorldMatrix;
						var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));

						gyro.Pitch = (float)transformedRotationVec.X;
						gyro.Yaw = (float)transformedRotationVec.Y;
						gyro.Roll = (float)transformedRotationVec.Z;
						gyro.GyroOverride = true;
					}
					Echo($"TickStep {tickStep}");
				}
				public void UpdateFrom(IMyRemoteControl controller)
				{
					ticker = (ticker + tickStep) % 360;
					var now = DateTime.UtcNow;
					tickTime = now - lastTickTime;
					lastTickTime = now;


					if (controller.TryGetPlanetElevation(MyPlanetElevation.Surface, out elevation) && elevation > -maxHeight * 2)
					{//do nothing in air
						moveLenght = moveAngle = moveAngle = turnAngle = pitch = yaw = 0;
						return;
					}
					heightMod = defaultHeight;
					pitch = 0;
					yaw = 0;

					prevMoveVector.X = moveVector.X;
					prevMoveVector.Y = moveVector.Y;

					Echo($"MoveIndicator {controller.MoveIndicator}");

					var currentWaypoint = controller.CurrentWaypoint;
					if (currentWaypoint.Coords == null) return;
					var targetLocation = currentWaypoint.Coords;
					var currentLocation = controller.GetPosition();
					var matrix = controller.WorldMatrix;
					var diff = targetLocation - currentLocation;
					var normDiff = diff;
					normDiff.Normalize();
					var dotForward = matrix.Forward.Dot(normDiff);
					var dotRight = matrix.Right.Dot(normDiff);
					var dotUp = matrix.Up.Dot(normDiff);
					if (dotUp > dotForward || diff.Length() < vehicleSize * 2)
					{
						List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
						controller.GetWaypointInfo(waypoints);
						waypoints.RemoveAt(0);
						waypoints.Add(currentWaypoint);//add current waypoint to last position
						controller.ClearWaypoints();
						waypoints.ForEach(w => controller.AddWaypoint(w.Coords, w.Name));
					}
					Echo($"dotForward {dotForward} \ndotLeft {dotRight}");

					moveVector.X = (float)dotForward;
					moveVector.Y = (float)dotRight;

					var gravityVec = controller.GetNaturalGravity();
					var gravityVecLength = gravityVec.Length();

					var referenceForward = controller.WorldMatrix.Forward;
					var referenceLeft = controller.WorldMatrix.Left;
					gravPitch = (float)Math.Atan(gravityVec.Dot(referenceForward) / gravityVecLength);
					gravRoll = (float)Math.Atan(gravityVec.Dot(referenceLeft) / gravityVecLength);

					var inputVec = controller.MoveIndicator;

					prevMoveVector.X = moveVector.X;
					prevMoveVector.Y = moveVector.Y;

					//Steering 
					moveVector.X -= inputVec.Z / 5f;

					//Turn 
					turnAngle = (float)(Math.Sign(inputVec.X) * Math.PI / 6f);

					moveVector *= .9f;//slowdown if no input
					if (moveVector.Length() > 1)
					{
						moveVector.Normalize();
					}
					if (moveVector.Length() < .1)
					{
						moveVector.X = 0;
						moveVector.Y = 0;
					}

					accelPitch = (float)Math.Atan2(moveVector.X - prevMoveVector.X, .2);
					accelRoll = (float)Math.Atan2(moveVector.Y - prevMoveVector.Y, .2);

					moveAngle = (float)Math.Atan2(moveVector.Y, moveVector.X);
					moveLenght = moveVector.Length();
					moveLenght -= moveLenght * Math.Abs(moveVector.Y) * .2f;//decrease move length for strafe

					var velocity = (float)controller.GetShipVelocities().LinearVelocity.Length();
					var expectedVelocity = Legs.Select(x => x.GetEstimatedSpeed(tickStep, tickTime)).Average();

					avgVelocity += (velocity - avgVelocity) / 10f;
					Echo($"Move forward {moveForward}");
					Echo($"Actual velocity     {Math.Round(avgVelocity, 4)}\nExpected velocity {Math.Round(expectedVelocity, 4)}");

					if (avgVelocity > .5 && avgVelocity > expectedVelocity * .8)
					{
						tickStep = Math.Min(tickStep + 5, maxTickStep);
					}
					else if (avgVelocity < .5 || avgVelocity < expectedVelocity * 2)
					{
						tickStep = Math.Max(tickStep - 5, 5);
					}

					landMult = .7f - Math.Min(tickStep / 50, .2f);
					Echo($"TickStep {tickStep}");
				}

				public void Apply()
				{
					float gravPitchMod = (float)Math.Sin(-gravPitch + accelPitch) * heightMod;
					float gravRollMod = (float)Math.Sin(gravRoll + accelRoll) * heightMod;
					Legs.ForEach(leg =>
					{

						float pitchMod = (float)Math.Sin(pitch) * leg.Position.X;

						float xMod = globalXmod + (float)Math.Sin(yaw) * leg.Position.Y;
						float yMod = globalYmod + (float)Math.Sin(Math.Min(Math.Max(-maxYaw, yaw), maxYaw)) * leg.Position.X * -leg.SideMult;

						leg.UpdateStep(moveAngle: moveAngle,
							stepHeightMult: (moveLenght != 0 || turnAngle != 0) && !slide ? 1 : 0,
							stepLengthMult: slide ? 0 : moveLenght,
							idleZ: heightMod,
							idleModX: xMod + gravPitchMod,
							idleModY: yMod + gravRollMod * leg.SideMult,
							idleModZ: globalZmod + pitchMod,
							turnAngle: slide ? 0 : turnAngle,
							landMult: landMult,
							overrideFriction: slide);
						leg.ApplyPhase(ticker, tickTime);
						if (slide)
						{
							leg.SetFriction(moveLenght != 0 ? 0 : 5);
						}
					});
				}
			}

			public class LegFactory
			{
				IMyGridTerminalSystem gridTerminalSystem;

				string innerJointGroup;

				IMyShipController controller;
				List<IMyMechanicalConnectionBlock> connections = new List<IMyMechanicalConnectionBlock>();
				List<IMyMotorStator> rotors = new List<IMyMotorStator>();
				List<IMyGyro> gyros = new List<IMyGyro>();
				List<IMyMotorStator> innerJointRotors = new List<IMyMotorStator>();
				List<IMyMotorSuspension> suspension = new List<IMyMotorSuspension>();
				List<IMyCubeBlock> allCubes = new List<IMyCubeBlock>();
				int phaseStep;

				Action<string> echo;

				List<string> debugMessages;
				public LegFactory(string innerJointGroup, IMyShipController controller, IMyGridTerminalSystem gridTerminalSystem, Action<string> echo, List<string> debugMessages, int phaseStep)
				{
					this.innerJointGroup = innerJointGroup;
					this.controller = controller;
					this.gridTerminalSystem = gridTerminalSystem;
					this.echo = echo;
					this.debugMessages = debugMessages;
					this.phaseStep = phaseStep;

					gridTerminalSystem.GetBlocksOfType(allCubes);
					gridTerminalSystem.GetBlocksOfType(rotors);
					gridTerminalSystem.GetBlocksOfType(gyros);
					gridTerminalSystem.GetBlocksOfType(suspension);
					gridTerminalSystem.GetBlocksOfType(connections);
					gridTerminalSystem.GetBlockGroupWithName(innerJointGroup)?.GetBlocksOfType<IMyMotorStator>(innerJointRotors);

				}

				public LegController GetLegs()
				{

					var groupedInnerJointRotors = innerJointRotors.GroupBy(x => x.TopGrid.EntityId);

					var vehicleCenter = innerJointRotors.Count >= 2
						? Utils.AverageBlockPosition(innerJointRotors)
						: new Vector3(0, 0, 0);

					var gridNode = new GridNode(controller, connections);

					//var legCandidates = gridNode.GetGridChain(new List<GridNode.Movement> { GridNode.Movement.Roll, GridNode.Movement.Roll, GridNode.Movement.Yaw }).ToList();
					//throw new Exception($"legCandidates {legCandidates.Count}");


					Matrix m;
					controller.Orientation.GetMatrix(out m);
					var rotation = Matrix.Invert(m) * Matrix.CreateFromYawPitchRoll((float)Math.PI * .5f, (float)Math.PI, -(float)Math.PI * .5f);//rotate to controller direction 

					var legs = groupedInnerJointRotors.Select(innerJoints =>
					{
						var innerGridId = innerJoints.Key;
						var innerJointPosition = Utils.AverageBlockPosition(innerJoints);
						var innerJointTopPosition = Utils.AverageBlockPosition(innerJoints.Select(x => x.Top));

						var midJoints = rotors.Where(x => x.CubeGrid.EntityId == innerGridId).GroupBy(x => x.TopGrid.EntityId).Single();
						var midGridId = midJoints.Key;
						var midJointPosition = Utils.AverageBlockPosition(midJoints);
						var midJointTopPosition = Utils.AverageBlockPosition(midJoints.Select(x => x.Top));

						var outerJoints = rotors.Where(x => x.CubeGrid.EntityId == midGridId).GroupBy(x => x.TopGrid.EntityId).Single();
						var outerGridId = outerJoints.Key;
						var outerJointTopPosition = Utils.AverageBlockPosition(outerJoints.Select(x => x.Top));

						var cubesInOuterGrid = allCubes.Where(x => x.CubeGrid.EntityId == outerGridId);
						var innerLength = Vector3.Distance(innerJointTopPosition, midJointPosition);
						var midLength = Vector3.Distance(midJointTopPosition, Utils.AverageBlockPosition(outerJoints));
						float outerLength = cubesInOuterGrid.Any()
							? cubesInOuterGrid.Max(x => Vector3.Distance(x.Position * x.CubeGrid.GridSize, outerJointTopPosition))
							: (float)outerJoints.First().TopGrid.WorldAABB.Size.Length();

						var legSuspension = suspension.Where(x => x != null && x.CubeGrid.EntityId == outerGridId).ToList();
						legSuspension.ForEach(x =>
						{
							x.Brake = true;
							x.SetValueFloat("Friction", 100);
							x.SetValueFloat("SafetyDetach", 5);
							x.SetValueBool("ShareInertiaTensor", false);
						});

						//use coordinates relative to controller 
						var legPosition = new Vector3I(Vector3.Transform(Utils.AverageBlockPosition(innerJoints) - vehicleCenter, rotation));
						if (Math.Abs(legPosition.Z) > .5)
						{
							throw new Exception($"Grid is not aligned\n rotation {rotation}\n legPosition {legPosition}\n");
						}

						//debugMessages.Add($"innerLength {innerLength},midLength {midLength},outerLength {outerLength},");
						var angleFromCenter = Math.Atan2(legPosition.Y, legPosition.X);
						var idlePositionOffset = (innerLength + (midLength + outerLength) * 0.3f);
						var sideMult = Math.Sign(legPosition.Y);
						var idlePosition = new Vector3
						{
							X = (float)(idlePositionOffset * Math.Cos(angleFromCenter)),
							Y = (float)(idlePositionOffset * Math.Sin(angleFromCenter) * sideMult),
							Z = -2,//TODO lowest Z in controller's grid
						};

						var sideChar = legPosition.Y > 0 ? "R" : "L";

						return new Leg(
							controller,
							vehicleCenter,
							new Joint($"{sideChar} I", controller, innerJoints.ToList(), innerLength, sideMult, gridNode),
							new Joint($"{sideChar} M", controller, midJoints.ToList(), midLength, sideMult, gridNode),
							new Joint($"{sideChar} O", controller, outerJoints.ToList(), outerLength, sideMult, gridNode),
							legSuspension,
							new Leg.Step
							{
								MoveAngle = 0,
								Height = Math.Abs(outerLength + midLength) * .4f,
								Length = (innerLength + midLength + outerLength) * 0.6f,
								PhaseShift = 0,
								IdlePosition = idlePosition,
								IdlePositionMod = new Vector3(0, 0, 0)
							},
							phaseStep,
							legPosition,
							legPosition.Y > 0 ? Base6Directions.Direction.Right : Base6Directions.Direction.Left,
							sideMult,
								this.echo);
					});

					var legController = new LegController(legs, this.echo, gyros);


					return legController;
				}
			}

			public class Leg
			{
				Action<string> Echo;

				public Vector3 VehicleCenter;
				public Base6Directions.Direction Orientation;
				public Vector3 Position;//relative to center 
				public float SideMult = 1;

				public Joint InnerJoint;
				public Joint MidJoint;
				public Joint OuterJoint;
				public List<IMyMotorSuspension> suspensions;

				Step step;
				int phaseStep;
				Vector2D currentPosition = new Vector2D();
				Vector2D estimatedPosition = new Vector2D();
				Vector2D velocity = new Vector2D();//indicate velocity of leg endpoint

				IDictionary<float, LegAngles> angles;

				public Leg(
					IMyShipController reference,
					Vector3 vehicleCenter,
					Joint innerJoint,
					Joint midJoint,
					Joint outerJoint,
					List<IMyMotorSuspension> suspensions,
					Step step,
					int phaseStep,
					Vector3I position,
					Base6Directions.Direction Orientation,
					float sideMult,
					Action<string> echo)
				{
					this.VehicleCenter = vehicleCenter;

					this.InnerJoint = innerJoint;
					this.MidJoint = midJoint;
					this.OuterJoint = outerJoint;
					this.suspensions = suspensions;
					this.step = step;
					this.phaseStep = phaseStep;
					this.Orientation = Orientation;
					this.SideMult = sideMult;
					this.Echo = echo;

					Position = position;
				}

				public void PreCalculateAngles()
				{
					angles = Enumerable.Range(0, 360 / phaseStep).ToDictionary(
						i => (float)i * phaseStep,
						phase => this.GetLegAngles(this.GetEsimatedLegPosition(phase * phaseStep)));
				}

				public void SetLimits()
				{
					PreCalculateAngles();
					var yawMin = angles.Min(x => x.Value.innerYaw);
					var yawMax = angles.Max(x => x.Value.innerYaw);
					var midPitchMin = angles.Min(x => x.Value.midPitch);
					var midPitchMax = angles.Max(x => x.Value.midPitch);
					var outerPitchMin = angles.Min(x => x.Value.outerPitch);
					var outerPitchMax = angles.Max(x => x.Value.outerPitch);

					InnerJoint.SetLimits(yawMin - .3f, yawMax + .3f);
					//crawler should be able to jump 
					MidJoint.SetLimits(midPitchMin - .1f, midPitchMax + .1f);
					OuterJoint.SetLimits(outerPitchMin - .1f, outerPitchMax + .1f);
				}

				public void SetFriction(float friction)
				{
					suspensions.ForEach(x =>
					{
						x?.SetValueFloat("Friction", friction);
					});
				}

				public void UpdateStep(float? moveAngle = null,
					float? phaseShift = null,
					float? height = null,
					float? length = null,
					float? stepHeightMult = null,
					float? stepLengthMult = null,
					float? idleZ = null,
					float? idleModX = null,
					float? idleModY = null,
					float? idleModZ = null,
					float? turnAngle = null,
					float? landMult = null,
					bool? overrideFriction = null
					)
				{
					this.step.MoveAngle = moveAngle * SideMult ?? this.step.MoveAngle;
					this.step.PhaseShift = phaseShift ?? this.step.PhaseShift;
					this.step.Height = height ?? this.step.Height;
					this.step.Length = length ?? this.step.Length;
					this.step.StepHeightMult = stepHeightMult ?? this.step.StepHeightMult;
					this.step.StepLengthMult = stepLengthMult ?? this.step.StepLengthMult;
					this.step.IdlePositionMod.X = idleModX ?? this.step.IdlePositionMod.X;
					this.step.IdlePositionMod.Y = idleModY ?? this.step.IdlePositionMod.Y;
					this.step.IdlePositionMod.Z = idleModZ ?? this.step.IdlePositionMod.Z;
					this.step.LandMult = landMult ?? this.step.LandMult;

					if (idleZ.HasValue && idleZ.Value != step.IdlePosition.Z)//calculate safe width offset
					{
						var newHeight = idleZ.Value;
						var angleFromCenter = Math.Atan2(Position.Y, Position.X);

						var maxZ = (MidJoint.Length + OuterJoint.Length) * 0.6;

						if (Math.Abs(newHeight) > maxZ)
						{
							newHeight = newHeight > 0 ? (float)maxZ : (float)-maxZ;
						}

						var vAngle = Math.Asin(MidJoint.Length / (newHeight - OuterJoint.Length));//angle of mid joint
						if (vAngle < .1) vAngle = .1;
						var offset = (float)((MidJoint.Length + OuterJoint.Length) * Math.Cos(vAngle) * .5f + InnerJoint.Length);
						if (float.IsNaN(offset))
						{
							throw new Exception($"Expacted y is NaN \n Z {idleZ.Value}\nOuterJoint.Length{OuterJoint.Length}\n vAngle{vAngle}\n\nangleFromCenter{angleFromCenter}");
						}

						step.IdlePosition.X = (float)(offset * Math.Cos(angleFromCenter));
						step.IdlePosition.Y = (float)(offset * Math.Sin(angleFromCenter) * SideMult);
						step.IdlePosition.Z = idleZ.Value;


					}

					if (turnAngle.HasValue && turnAngle.Value != 0)
					{
						var angle = turnAngle.Value;
						bool isMoving = Math.Abs(step.StepLengthMult) > .1f;
						if (isMoving)
						{
							this.step.MoveAngle = (float)Math.Atan2(Math.Tan(angle * SideMult) * step.IdlePosition.X, step.IdlePosition.Y);
							this.step.StepLengthMult = (float)Math.Min(step.Length, step.Length - step.Length * Math.Sin(angle * SideMult)) / step.Length;
						}
						else//turn in place
						{
							var stepAngle = MathHelper.ToDegrees((float)Math.Atan2(step.IdlePosition.Y * SideMult + Position.Y, step.IdlePosition.X + Position.X));
							stepAngle += (float)(Math.Sign(angle) * Math.PI / 2f);
							stepAngle *= SideMult;
							this.step.MoveAngle = stepAngle;
							this.step.StepLengthMult = .5f;
							this.step.StepHeightMult = .5f;
						}

						step.TurnAngle = turnAngle.Value;
					}

					SetLimits();
				}

				public float ApplyPhase(float phase, TimeSpan tickTime)
				{
					var estimatedPosition3 = GetEsimatedLegPosition(phase);
					var previousVelocity = velocity;
					velocity = currentPosition - estimatedPosition;


					var currentAngles = GetLegAngles(estimatedPosition3);// angles[currentPhase]; 

					if (HexapodWorker.DEBUG)
					{
						Echo($"Orientation\n {Orientation}\n");
						Echo($"Position\n {Position}\n");
						Echo($"Step\n {step}\n");
						Echo($"estimatedPosition\n {estimatedPosition}\n");

						Echo($"SideMult\n {SideMult}");
						Echo($"CurrentAngles\n {currentAngles}\n");
						Echo($"----------------------------------------\n");
					}

					SetAngles(currentAngles, tickTime, step.StepLengthMult > .1f ? 1f : .3f);
					currentPosition = estimatedPosition;

					if (!step.overrideFriction && velocity.X >= 0 && previousVelocity.X < 0)//move forward, maximize friction
					{
						SetFriction(100);
					}
					else if (!step.overrideFriction && velocity.X < 0 && previousVelocity.X >= 0)
					{
						SetFriction(0);
					}

					return Math.Abs(InnerJoint.TargetAngle - InnerJoint.CurrentAngle);
				}

				public void SetAngles(LegAngles angles, TimeSpan tickTime, float velocityMult = 1f)
				{
					InnerJoint.SetAngle(angles.innerYaw, tickTime, velocityMult);
					MidJoint.SetAngle(angles.midPitch, tickTime, velocityMult);
					OuterJoint.SetAngle(angles.outerPitch, tickTime, velocityMult);
				}

				public void Straighten(TimeSpan tickTime)
				{
					InnerJoint.SetAngle(0, tickTime);
					MidJoint.SetAngle(0.5f, tickTime);
					OuterJoint.SetAngle(-0.5f, tickTime);
				}

				public Vector3 GetLegPosition()
				{
					var sinYaw = Math.Sin(this.InnerJoint.CurrentAngle);
					var cosYaw = Math.Cos(this.InnerJoint.CurrentAngle);

					var res = new Vector3(
						this.InnerJoint.Length * sinYaw,
						this.InnerJoint.Length * cosYaw,
						0
					);
					var dxMid = Math.Cos(this.MidJoint.CurrentAngle) * this.MidJoint.Length;
					var dzMid = Math.Sin(this.MidJoint.CurrentAngle) * this.MidJoint.Length;

					res += new Vector3(
						dxMid * sinYaw,
						dxMid * cosYaw,
						dzMid
					);

					var c = this.MidJoint.CurrentAngle + this.OuterJoint.CurrentAngle;
					var dxOuter = Math.Cos(c) * this.OuterJoint.Length;
					var dzOuter = Math.Sin(c) * this.OuterJoint.Length;

					res += new Vector3(
						dxOuter * sinYaw,
						dxOuter * cosYaw,
						dzOuter
					);
					return res;
				}

				public Vector3 GetEsimatedLegPosition(float phase)
				{
					var currentPhase = (phase + step.PhaseShift) % 360;

					//transform phase
					var landPhase = 360 * (1 - step.LandMult);
					var airPhase = 360 * step.LandMult;
					currentPhase = currentPhase < landPhase
						? currentPhase / landPhase * 180
						: (currentPhase - landPhase) / airPhase * 180 + 180;

					estimatedPosition.X = ((currentPhase < 180 ? currentPhase : 180 - currentPhase % 180) / 180 - 0.5) * this.step.Length * step.StepLengthMult;//linear movement along step direction 

					estimatedPosition.Y = Math.Max(Math.Sin(currentPhase * Math.PI / 180), -0.2) * this.step.Height / 1.2 * step.StepHeightMult;
					//var stepZ = Math.Max((currentPhase > 180 ? Math.Cos(phase * Math.PI / 360) + 0.9 : Math.Cos((phase - 120) * Math.PI / 360)) * .9 - .1, 0) * step.Height * step.StepLengthMult;
					//estimatedPosition.Y = Math.Max((Math.Sin(currentPhase * Math.PI / 180) + Math.Sin(currentPhase * Math.PI / 180 - Math.PI / 6)), -.2) / 2 * this.step.Height * step.StepHeightMult;

					var x = this.step.IdlePosition.X + this.step.IdlePositionMod.X + estimatedPosition.X * Math.Cos(step.MoveAngle);
					var y = this.step.IdlePosition.Y + this.step.IdlePositionMod.Y + estimatedPosition.X * Math.Sin(step.MoveAngle);

					return new Vector3D { X = x, Y = y, Z = estimatedPosition.Y + step.IdlePosition.Z + step.IdlePositionMod.Z };
				}

				public float GetEstimatedSpeed(int phaseStep, TimeSpan tickTime)//move distance per tick
				{
					return step.Length * step.StepLengthMult * phaseStep / (360 * step.LandMult) / tickTime.Milliseconds * 1000f;//WTF
				}

				public LegAngles GetLegAngles(Vector3D esimatedLegPosition)
				{
					var yawRad = (float)Math.Atan2(esimatedLegPosition.X, esimatedLegPosition.Y);


					var dx = Math.Sqrt(esimatedLegPosition.X * esimatedLegPosition.X + esimatedLegPosition.Y * esimatedLegPosition.Y);
					if (yawRad > Math.PI / 2f || yawRad < -Math.PI / 2f)
					{
						yawRad += (float)Math.PI * Math.Sign(yawRad);
						yawRad = Utils.NormalizeAngle(yawRad);
						dx = -dx;
					}

					dx = dx - this.InnerJoint.Length;
					var dz = esimatedLegPosition.Z;


					var hyp = Math.Sqrt(dx * dx + dz * dz);
					var pitchToTarget = Math.Atan2(dz, dx);
					if (hyp > OuterJoint.Length + MidJoint.Length)//too far
					{
						return new LegAngles { innerYaw = (float)yawRad, midPitch = (float)pitchToTarget, outerPitch = 0 };
					}
					else
					if (hyp < Math.Abs(OuterJoint.Length - MidJoint.Length))//too close
					{
						return new LegAngles { innerYaw = (float)yawRad, midPitch = (float)-Math.PI / 2, outerPitch = (float)Math.PI * 0.9f };
					}


					var midPitch = Math.Acos((OuterJoint.Length * OuterJoint.Length - MidJoint.Length * MidJoint.Length - hyp * hyp) / (-2f * MidJoint.Length * hyp)) + pitchToTarget;
					var outerPitch = Math.Acos((hyp * hyp - MidJoint.Length * MidJoint.Length - OuterJoint.Length * OuterJoint.Length) / (-2f * MidJoint.Length * OuterJoint.Length)) - Math.PI;

					var res = new LegAngles { innerYaw = (float)yawRad, midPitch = (float)midPitch, outerPitch = (float)outerPitch };

					if (double.IsNaN(yawRad) || double.IsNaN(midPitch) || double.IsNaN(outerPitch))
						throw new Exception($"Something is NaN\n {esimatedLegPosition}\n{res}\n");

					return res;
				}

				public class Step
				{
					public float MoveAngle;
					public float Length;
					public float Height;
					public float PhaseShift;
					public float StepHeightMult;
					public float StepLengthMult;
					public float LandMult = .5f;//proportion of time when leg touch ground

					public float TurnAngle;

					public bool overrideFriction = false;

					public Vector3 IdlePosition;//relative to innerJointPosition 
					public Vector3 IdlePositionMod;//relative to IdlePosition - used to quick modify step postion, for exmaple for jump 

					public override string ToString()
					{
						return $"Angle {MoveAngle}\nLength {Length}\nHeight {Height}\nStepLengthMult {StepLengthMult}\nStepHeightMult {StepHeightMult}\nPhaseShift {PhaseShift}\nIdlePosition {IdlePosition}\nIdlePositionMod {IdlePositionMod}";
					}
				}

				public class LegAngles
				{
					public float innerYaw;
					public float midPitch;
					public float outerPitch;

					public override string ToString()
					{
						return $"innerYaw {MathHelper.ToDegrees(innerYaw)}\nmidPitch {MathHelper.ToDegrees(midPitch)}\nouterPitch {MathHelper.ToDegrees(outerPitch)}";
					}
				}
			}
			public class Joint
			{
				public string name;
				public List<IMyMotorStator> Rotors;
				List<IMyMotorStator> rotorsClockwise;
				List<IMyMotorStator> rotorsCounterClockwise;
				public float Length { get; protected set; }
				public Vector3 Position { get; protected set; }

				public float TargetAngle = 0;
				public float previousAngle = 0;
				public TimeSpan previousTickTime = TimeSpan.FromMilliseconds(0);
				public float previousVelocity = 0;
				public const float speedDebtTreshold = 0.2f;
				public const float limitBoost = 0.005f;

				public readonly float millisecondsInSecond = 1000;

				float sideMult;
				float angleMin = -(float)Math.PI;
				float angleMax = (float)Math.PI;

				float EffectiveAngleMin
				{
					get
					{
						return Math.Min(angleMin * sideMult, angleMax * sideMult);
					}
				}
				float EffectiveAngleMax
				{
					get
					{
						return Math.Max(angleMin * sideMult, angleMax * sideMult);
					}
				}

				public float CurrentAngle
				{
					get
					{
						return NormalizeAngle(rotorsClockwise?.FirstOrDefault()?.Angle ?? -rotorsCounterClockwise?.First()?.Angle ?? 0) * sideMult;
					}
				}
				public float CurrentVelocity
				{
					get
					{
						return (CurrentAngle - previousAngle) / previousTickTime.Milliseconds * millisecondsInSecond;//rad per second 
					}
				}

				public Joint(string name, IMyShipController reference, List<IMyMotorStator> rotors, float length, float sideMult)
				{
					this.name = name;
					this.Length = length;
					this.sideMult = sideMult;

					rotorsClockwise = rotors.Where(x =>
					{
						return reference.WorldMatrix.Down.Dot(x.WorldMatrix.Up) > 0.5
						|| reference.WorldMatrix.Forward.Dot(x.WorldMatrix.Up) > 0.1;
						//|| reference.WorldMatrix.Left.Dot(x.WorldMatrix.Up) * sideMult > 0.1;
					}).ToList();
					rotorsCounterClockwise = rotors.Where(x =>
					{
						return reference.WorldMatrix.Up.Dot(x.WorldMatrix.Up) > 0.5
						|| reference.WorldMatrix.Backward.Dot(x.WorldMatrix.Up) > 0.1;
						//|| reference.WorldMatrix.Right.Dot(x.WorldMatrix.Up) * sideMult > 0.1;
					}).ToList();

					if (!rotorsClockwise.Any() && !rotorsCounterClockwise.Any())
					{
						throw new Exception($"Joint have no rotors in correct orientation");
					}

					rotors.ForEach(rotor =>
					{
						if (rotor.LowerLimitRad < -Math.PI)
						{
							rotor.LowerLimitRad = (float)-Math.PI;
						}
						if (rotor.UpperLimitRad > Math.PI)
						{
							rotor.UpperLimitRad = (float)Math.PI;
						}
					});

					rotorsClockwise?.ForEach(rotor =>
					{
						rotor.CustomName = $"Rotor {name} CW";

					});
					rotorsCounterClockwise?.ForEach(rotor =>
					{
						rotor.CustomName = $"Rotor {name} CCW";
					});
				}

				public Joint(string name, IMyShipController reference, List<IMyMotorStator> rotors, float length, float sideMult, GridNode node)
				{
					this.name = name;
					this.Length = length;
					this.sideMult = sideMult;

					var referenceOrientation = node.GetRelativeOrientation(reference);

					rotorsClockwise = rotors.Where(x =>
					{
						var orientation = node.GetRelativeOrientation(x);
						return referenceOrientation.Down.Dot(orientation.Up) > .9
						 || referenceOrientation.Backward.Dot(orientation.Up) > .9
						 || referenceOrientation.Left.Dot(orientation.Up) > .9;
					}).ToList();
					rotorsCounterClockwise = rotors.Where(x =>
					{
						var orientation = node.GetRelativeOrientation(x);
						return referenceOrientation.Up.Dot(orientation.Up) > .9
						 || referenceOrientation.Forward.Dot(orientation.Up) > .9
						 || referenceOrientation.Right.Dot(orientation.Up) > .9;
					}).ToList();

					if (!rotorsClockwise.Any() && !rotorsCounterClockwise.Any())
					{
						throw new Exception($"Joint have no rotors in correct orientation");
					}

					rotors.ForEach(rotor =>
					{
						if (rotor.LowerLimitRad < -Math.PI)
						{
							rotor.LowerLimitRad = (float)-Math.PI;
						}
						if (rotor.UpperLimitRad > Math.PI)
						{
							rotor.UpperLimitRad = (float)Math.PI;
						}
					});

					rotorsClockwise?.ForEach(rotor =>
					{
						rotor.CustomName = $"Rotor {name} CW";

					});
					rotorsCounterClockwise?.ForEach(rotor =>
					{
						rotor.CustomName = $"Rotor {name} CCW";
					});
				}


				public void SetAngle(float angle, TimeSpan tickTime, float velocityMult = 1f)
				{
					TargetAngle = angle;
					angle = NormalizeAngle(sideMult * TargetAngle);
					var currentVelocity = CurrentVelocity;
					var currentAngle = NormalizeAngle(CurrentAngle * sideMult);

					previousAngle = currentAngle;
					previousTickTime = tickTime;

					if (float.IsNaN(angle)) return;

					var diff = angle - currentAngle;
					//var targetVelocity = (float)Math.Min(Math.Max(-Math.PI, diff * tickTime.Milliseconds / 100), Math.PI); 
					var targetVelocity = diff / (float)tickTime.Milliseconds * millisecondsInSecond;
					var speedDebt = targetVelocity - currentVelocity;

					targetVelocity = targetVelocity * velocityMult;
					if (float.IsNaN(targetVelocity)) return;

					rotorsClockwise?.ForEach(rotor =>
					{

						rotor.TargetVelocityRad = targetVelocity;

						if (targetVelocity > speedDebtTreshold)
						{//kick lower limit
							rotor.LowerLimitRad = rotor.Angle + limitBoost;
						}
						else if (Math.Abs(rotor.LowerLimitRad - EffectiveAngleMin) > .1)
						{//restore limit if needed
							rotor.LowerLimitRad = Math.Min(EffectiveAngleMin, rotor.Angle);
						}
						if (targetVelocity < -speedDebtTreshold)
						{//kick upper limit
							rotor.UpperLimitRad = rotor.Angle - limitBoost;
						}
						else if (Math.Abs(rotor.UpperLimitRad - EffectiveAngleMax) > .1)
						{//restore limit if needed
							rotor.UpperLimitRad = Math.Max(EffectiveAngleMax, rotor.Angle);
						}
						if (HexapodWorker.DEBUG)
						{
							rotor.CustomData =
								$"CW\n" +
								$"Rotor angle {MathHelper.ToDegrees(rotor.Angle)}\n" +
								$"Target angle {MathHelper.ToDegrees(angle)}\n " +
								$"Target Velocity {MathHelper.ToDegrees(targetVelocity)}\n " +
								$"Velocity {MathHelper.ToDegrees(diff)}";
							rotor.CustomName = $"Rotor {name} CW  {Math.Round(MathHelper.ToDegrees(angle), 2)} {Math.Round(rotor.TargetVelocityRPM, 2)}";
						}
					});
					rotorsCounterClockwise?.ForEach(rotor =>
					{
						rotor.TargetVelocityRad = -targetVelocity;
						if (-targetVelocity > speedDebtTreshold)
						{//kick lower limit
							rotor.LowerLimitRad = rotor.Angle + limitBoost;
						}
						else if (Math.Abs(rotor.LowerLimitRad + EffectiveAngleMax) > .1)
						{//restore limit if needed
							rotor.LowerLimitRad = Math.Min(-EffectiveAngleMax, rotor.Angle);
						}
						if (-targetVelocity < -speedDebtTreshold)
						{//kick upper limit
							rotor.UpperLimitRad = rotor.Angle - limitBoost;
						}
						else if (Math.Abs(rotor.UpperLimitRad + EffectiveAngleMin) > .1)
						{//restore limit if needed
							rotor.UpperLimitRad = Math.Max(-EffectiveAngleMin, rotor.Angle);
						}
						if (HexapodWorker.DEBUG)
						{
							rotor.CustomData =
								$"CCW\n " +
								$"Rotor angle {MathHelper.ToDegrees(rotor.Angle)}\n " +
								$"Target angle {MathHelper.ToDegrees(-angle)}\n" +
								$"Target Velocity {MathHelper.ToDegrees(-targetVelocity)}\n " +
								$"Velocity {MathHelper.ToDegrees(diff)}";
							rotor.CustomName = $"Rotor {name} CCW {Math.Round(MathHelper.ToDegrees(-angle), 2)} {Math.Round(rotor.TargetVelocityRPM, 2)}";
						}
					});
				}

				public void SetLimits(float min, float max)
				{
					angleMin = NormalizeAngle(min);
					angleMax = NormalizeAngle(max);
					rotorsClockwise?.ForEach(rotor =>
					{
						rotor.LowerLimitRad = new[] { min * sideMult, max * sideMult, rotor.Angle }.Min();
						rotor.UpperLimitRad = new[] { min * sideMult, max * sideMult, rotor.Angle }.Max();
					});
					rotorsCounterClockwise?.ForEach(rotor =>
					{
						rotor.LowerLimitRad = new[] { -min * sideMult, -max * sideMult, rotor.Angle }.Min();
						rotor.UpperLimitRad = new[] { -min * sideMult, -max * sideMult, rotor.Angle }.Max();
					});
				}

				float NormalizeAngle(float angle)
				{
					return Utils.NormalizeAngle(angle);
				}
			}

			public class GridNode
			{
				List<GridNode> ChildNodes = new List<GridNode>();
				GridNode ParentGrid;

				public IMyCubeGrid Grid;
				IMyCubeBlock Reference;

				IMyMechanicalConnectionBlock ConnectionToParent;
				IEnumerable<IMyMechanicalConnectionBlock> ConnectionsToParent;
				public IEnumerable<IMyPistonBase> PistonsToParent
				{
					get
					{
						return ConnectionsToParent.Where(x => x is IMyPistonBase).Select(x => x as IMyPistonBase);
					}
				}
				public IEnumerable<IMyMotorStator> RotorsToParent
				{
					get
					{
						return ConnectionsToParent.Where(x => x is IMyMotorStator).Select(x => x as IMyMotorStator);
					}
				}
				public IEnumerable<IMyMotorStator> RotorsCW
				{
					get
					{
						return ConnectionsToParent.Where(x => x is IMyMotorStator).Select(x => x as IMyMotorStator);
					}
				}
				public IEnumerable<IMyMotorStator> RotorsCCW
				{
					get
					{
						return ConnectionsToParent.Where(x => x is IMyMotorStator).Select(x => x as IMyMotorStator);
					}
				}

				public GridNode(IMyCubeBlock reference, IEnumerable<IMyMechanicalConnectionBlock> allConnections)
				{
					this.Grid = reference.CubeGrid;
					this.Reference = reference;
					this.ChildNodes = GetChildGrids(Grid, allConnections).Select(x => new GridNode(x, allConnections, Grid)).ToList();
					this.ChildNodes.ForEach(x => x.ParentGrid = this);
				}

				GridNode(IMyCubeGrid grid, IEnumerable<IMyMechanicalConnectionBlock> allConnections, IMyCubeGrid parent)
				{
					this.Grid = grid;
					this.ConnectionsToParent = allConnections.Where(x => x.CubeGrid.EntityId == parent.EntityId && x.TopGrid.EntityId == grid.EntityId).ToList();
					this.ConnectionToParent = ConnectionsToParent.FirstOrDefault();
					this.ChildNodes = GetChildGrids(grid, allConnections).Select(x => new GridNode(x, allConnections, grid)).ToList();
					this.ChildNodes.ForEach(x => x.ParentGrid = this);
				}

				public IEnumerable<IMyMechanicalConnectionBlock> GetConnectionsToChild(GridNode node)
				{
					return ChildNodes.Where(x => x.Grid.EntityId == node.Grid.EntityId).Select(x => x.ConnectionToParent);
				}

				public IMyCubeBlock RootReference
				{
					get
					{
						return this.Reference ?? ParentGrid.RootReference;
					}
				}

				public Movement MovementToParent
				{
					get
					{
						if (PistonsToParent.Any()) return Movement.Extend;
						bool isRotors = RotorsToParent.Any();
						var orientation = ParentGrid.GetRelativeOrientation(RotorsToParent.First());
						Matrix rootOrientation;
						RootReference.Orientation.GetMatrix(out rootOrientation);
						if (YawCriteria(orientation, rootOrientation)) return Movement.Yaw;
						if (PitchCriteria(orientation, rootOrientation)) return Movement.Pitch;
						if (RollCriteria(orientation, rootOrientation)) return Movement.Roll;
						return Movement.Unknown;
					}
				}

				static bool YawCwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Up) > .9;
				}
				static bool YawCcwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Down) > .9;
				}
				static bool PitchCwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Right) > .9;
				}
				static bool PitchCcwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Left) > .9;
				}
				static bool RollCwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Forward) > .9;
				}
				static bool RollCcwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return rotorOrientation.Up.Dot(rootOrientation.Backward) > .9;
				}

				static bool CwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return YawCwCriteria(rotorOrientation, rootOrientation) || PitchCwCriteria(rotorOrientation, rootOrientation) || RollCwCriteria(rotorOrientation, rootOrientation);
				}
				static bool CcwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return YawCcwCriteria(rotorOrientation, rootOrientation) || PitchCcwCriteria(rotorOrientation, rootOrientation) || RollCcwCriteria(rotorOrientation, rootOrientation);
				}

				static bool YawCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return YawCwCriteria(rotorOrientation, rootOrientation) || YawCcwCriteria(rotorOrientation, rootOrientation);
				}
				static bool PitchCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return PitchCwCriteria(rotorOrientation, rootOrientation) || PitchCcwCriteria(rotorOrientation, rootOrientation);
				}
				static bool RollCriteria(Matrix rotorOrientation, Matrix rootOrientation)
				{
					return RollCwCriteria(rotorOrientation, rootOrientation) || RollCcwCriteria(rotorOrientation, rootOrientation);
				}

				public IEnumerable<Movement> MovementChain
				{
					get
					{
						if (ParentGrid == null) return new List<Movement>();
						return (new[] { MovementToParent }).Concat(ParentGrid.MovementChain);
					}
				}

				public IEnumerable<GridNode> PathToRoot
				{
					get
					{
						return (new List<GridNode>() { this }).Concat(ParentGrid?.PathToRoot ?? new GridNode[0]);
					}
				}

				public IEnumerable<GridNode> AllGrids()
				{
					return (new List<GridNode>() { this }).Concat(ChildNodes.SelectMany(x => x.AllGrids()));
				}

				public GridNode GetGridForBlock(IMyCubeBlock block)
				{
					return AllGrids().Where(x => block.CubeGrid.EntityId == x.Grid.EntityId).FirstOrDefault();
				}

				public IEnumerable<GridNode> GetGridChain(IEnumerable<Movement> expectedChain)
				{
					return AllGrids().Where(x => x.MovementChain.SequenceEqual(expectedChain));
				}

				public Vector3 GetRelativePosition(IMyCubeBlock block)
				{
					var grid = GetGridForBlock(block);
					if (grid == null)
					{
						throw new Exception("This block is not in child grid");
					}
					if (grid == this)
					{
						return GetRelativePosition(block.Position, Reference);
					}

					return grid.GetRelativePosition(block.Position);
				}

				Vector3 GetRelativePosition(Vector3I pos)
				{
					pos = TransformPosition(pos, ConnectionToParent, ParentGrid?.Reference);
					return ParentGrid?.GetRelativePosition(pos) ?? pos;
				}

				public Matrix GetRelativeOrientation(IMyCubeBlock block)
				{
					if (block == null) throw new Exception("Block is null");
					var grid = GetGridForBlock(block);
					if (grid == null)
					{
						throw new Exception($"This block is not in child grid\n{block.Name}{block.CubeGrid.CustomName}");
					}

					Matrix orientation;
					block.Orientation.GetMatrix(out orientation);

					return grid.GetRelativeOrientation(orientation);
				}

				Matrix GetRelativeOrientation(Matrix orientation)
				{
					orientation = TransformOrientation(orientation, ConnectionToParent, ParentGrid?.Reference);
					return ParentGrid?.GetRelativeOrientation(orientation) ?? orientation;
				}

				public override string ToString()
				{
					return $"Grid {Grid.CustomName}\n ChildNodes:\n {string.Join("\n", ChildNodes.Select(x => x.ToString()))}";
				}

				public static Action<string> Echo = x => { };
				static IEnumerable<IMyCubeGrid> GetChildGrids(IMyCubeGrid grid, IEnumerable<IMyMechanicalConnectionBlock> connections)
				{
					return connections.Where(x => x.CubeGrid.EntityId == grid.EntityId).Select(x => x.TopGrid);
				}
				static Vector3I GetRelativePosition(Vector3I pos, IMyCubeBlock reference)
				{
					var referencePos = reference.Position;
					var diff = pos - referencePos;
					Matrix rotation;
					reference.Orientation.GetMatrix(out rotation);
					rotation = Matrix.Invert(rotation);
					var res = Vector3.Transform(diff, rotation);
					return new Vector3I(res);
				}
				static Vector3I TransformPosition(Vector3I src, IMyMechanicalConnectionBlock connection, IMyCubeBlock reference = null)
				{
					if (connection == null) return src;
					var top = connection.Top;
					var pos = GetRelativePosition(src, top);

					Matrix rotation;
					connection.Orientation.GetMatrix(out rotation);
					//rotation = Matrix.Invert(rotation);

					var translation = reference != null ? connection.Position - reference.Position : connection.Position;
					rotation.Translation = translation;
					pos = new Vector3I(Vector3.Transform(pos, rotation));

					if (reference != null)
					{
						pos = AlignTo(pos, reference);
					}

					return pos;
				}
				static Matrix TransformOrientation(Matrix src, IMyMechanicalConnectionBlock connection, IMyCubeBlock reference = null)
				{
					if (connection == null) return src;
					var top = connection.Top;
					Matrix orientationTop;
					top.Orientation.GetMatrix(out orientationTop);
					orientationTop = Matrix.Invert(orientationTop);

					Matrix orientationConnection;
					connection.Orientation.GetMatrix(out orientationConnection);

					src *= orientationTop;
					src *= orientationConnection;
					if (reference != null)
					{
						Matrix orientationReference;
						reference.Orientation.GetMatrix(out orientationReference);
						orientationReference = Matrix.Invert(orientationReference);
						src *= orientationReference;
					}

					return src;
				}
				static Vector3I AlignTo(Vector3I pos, IMyCubeBlock reference)
				{
					Matrix rotation;
					reference.Orientation.GetMatrix(out rotation);
					rotation = Matrix.Invert(rotation);
					var res = Vector3.Transform(pos, rotation);
					return new Vector3I(res);
				}

				public enum Movement
				{
					Extend,
					Yaw,
					Pitch,
					Roll,
					Unknown
				}
			}

			public static class Utils
			{
				public static float ToRadians(float input)
				{
					return (float)Math.PI * input / 180f;
				}
				public static float ToDegrees(float input)
				{
					return input * 180 / (float)Math.PI;
				}

				public static Vector3 Average(IEnumerable<Vector3> input)
				{
					return new Vector3
					{
						X = (float)input.Average(p => p.X),
						Y = (float)input.Average(p => p.Y),
						Z = (float)input.Average(p => p.Z),
					};
				}
				public static Vector3 AverageBlockPosition(IEnumerable<IMyCubeBlock> input)
				{
					return Utils.Average(input.Select(x => x.Position * x.CubeGrid.GridSize));
				}

				public static float NormalizeAngle(float angle)
				{
					while (angle <= -Math.PI) angle += MathHelper.TwoPi;
					while (angle > Math.PI) angle -= MathHelper.TwoPi;
					return angle;
				}
			}
		}

#if DEBUG
	}
}
#endif
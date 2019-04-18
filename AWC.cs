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

namespace AWC
{
	public sealed class Program : MyGridProgram
	{
#endif
		IMyTextPanel debugScreen;
		VehicleController worker;
		Action<string> echo;

		Program()
		{
			debugScreen = GridTerminalSystem.GetBlockWithName("WDebug") as IMyTextPanel;
			echo = debugScreen != null
				? t => { debugScreen.WriteText(t + "\n", true); Me.GetSurface(0).WriteText(t + "\n", true); Echo(t); }
			: (Action<string>)(t => { Me.GetSurface(0).WriteText(t + "\n", true); Echo(t); });
			try
			{
				worker = new VehicleController(GridTerminalSystem, echo);
			}
			catch (Exception e)
			{
				echo(e.Message);
				echo(e.Source);
				echo(e.StackTrace);
				throw;
			}
			Runtime.UpdateFrequency = UpdateFrequency.Update1;
		}

		void Main(string argument, UpdateType updateType)
		{
			debugScreen?.WriteText("");
			Me.GetSurface(0).WriteText("");
			try
			{
				worker.Tick(argument);
				echo(worker.walkerData.ticker.ToString());
				echo($"Cruise control {(worker.walkerData.CruiseMode ? "On" : "Off")}");
			}
			catch (Exception e)
			{
				Echo(e.Message);
				Echo(e.Source);
				Echo(e.StackTrace);
				Me.Enabled = false;
			}
		}

		public class VehicleController
		{
			public static bool DEBUG = false;
			IMyGridTerminalSystem gridTerminalSystem;
			List<IMyShipController> controllers = new List<IMyShipController>();

			IMyShipController GetCurrentController()
			{
				return controllers.FirstOrDefault(x => x.CanControlShip && (x.IsUnderControl || ((x as IMyRemoteControl)?.IsAutoPilotEnabled ?? false)));
			}

			public VehicleData walkerData = new VehicleData();
			public Ticker ticker = new Ticker();

			List<string> debugMessages = new List<string>();
			IDictionary<string, Action> arguments;

			public static Action<string> Echo;

			public Actor currentActor;

			public VehicleController(IMyGridTerminalSystem gridTerminalSystem, Action<string> echo)
			{
				this.gridTerminalSystem = gridTerminalSystem;
				Echo = echo;

				arguments = new Dictionary<string, Action> {
					{"", () => { } },
				};

				currentActor = new IdleActor();
			}

			public void Tick(string argument)
			{
				if (!string.IsNullOrWhiteSpace(argument))
				{
					arguments[argument]();
					return;
				}

				if (walkerData.CanWork)
				{
					walkerData.Update();
					currentActor = currentActor.Do(walkerData);
				}
				else if (TryInit(walkerData))
				{
					currentActor = new IdleActor();
				}
				debugMessages.ForEach(Echo);
			}

			public bool TryInit(VehicleData data)
			{
				var controllers = new List<IMyShipController>();
				gridTerminalSystem.GetBlocksOfType(controllers, x => x.CanControlShip);
				var controllersUnderControl = controllers.Where(x => x.IsUnderControl || ((x as IMyRemoteControl)?.IsAutoPilotEnabled ?? false)).ToList();

				if (controllers.Count == 1)
				{
					data.currentController = controllers[0];
				}
				else if (controllersUnderControl.Any())
				{
					data.currentController = controllersUnderControl[0];
				}
				else
				{
					return false;
				}

				var gyros = new List<IMyGyro>();
				gridTerminalSystem.GetBlocksOfType(gyros, x => x.CubeGrid.EntityId == data.currentController.CubeGrid.EntityId);
				data.gc = new GyroController(data.currentController, gyros);
				data.gs = new GravityScanner(data.currentController);

				var connections = new List<IMyMechanicalConnectionBlock>();
				gridTerminalSystem.GetBlocksOfType(connections);
				data.gridNode = new GridNode(gridTerminalSystem, data.currentController, connections);

				var cameras = new List<IMyCameraBlock>();
				gridTerminalSystem.GetBlocksOfType(cameras);
				data.scanner = new SurfaceScanner(data.gridNode, cameras);

				var wheels = new List<IMyMotorSuspension>();
				gridTerminalSystem.GetBlocksOfType(wheels);
				data.wheels = Wheel.GetWheels(data.gridNode, wheels).ToList();

				data.centerOfMass = data.gridNode.GetRelativePosition(data.currentController.CubeGrid.WorldToGridInteger(data.currentController.CenterOfMass));

				return true;
			}

			public class VehicleData
			{

				public bool CanWork { get { return currentController != null; } }
				public bool UnderControl { get { return currentController.IsUnderControl; } }

				public Ticker ticker = new Ticker();
				public GyroController gc;
				public GravityScanner gs;
				public SurfaceScanner scanner;
				public IMyShipController currentController;
				public GridNode gridNode;

				public List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
				public List<Wheel> wheels = new List<Wheel>();

				public Vector3 moveVector;
				public Vector2 moveVector2D = new Vector2();
				public Vector2 alignedVelocity2D = new Vector2();
				public Vector2 angularVelocity2D = new Vector2();//pitch, roll
				public Vector2 avgVelocity2D = new Vector2();
				public Vector3 rotationIndidcator = new Vector3();//pitch,yaw,roll
				public Vector3 centerOfMass = new Vector3();
				public Vector3 centerOfMassOffset { get { return centerOfMass - wheelCenter; } }//diff from center of mass to average leg postion
				public Vector3 wheelCenter;
				public float idleHeight;
				public float currentHeight;

				//expected movement
				public float moveAngle;
				public float moveLenght;
				//actual movement
				public float currentMoveAngle;
				public float currentMmoveLenght;

				public float maxPitch = MathHelper.PiOver4 * .5f;
				public float pitch;
				public float yaw;

				public double stepLength = 5;
				public double stepLengthMult = 0;
				public double stepHeight = 2;
				public Vector3 offset;

				DateTime pressTime = DateTime.Now;
				bool pressed;
				bool unpressed;
				public bool CruiseMode;

				void ToggleCruiseMode()
				{
					if (moveVector.Z < -.5 && !pressed && !CruiseMode)
					{
						pressed = true;
						pressTime = DateTime.Now.AddSeconds(1);
					}
					if (moveVector.Z == 0 && pressed)
					{
						pressed = false;
						unpressed = true;
					}
					if (DateTime.Now > pressTime)
					{
						unpressed = false;
					}
					if (moveVector.Z < -.5 && unpressed && DateTime.Now < pressTime)
					{
						unpressed = false;
						CruiseMode = true;
					}
					if (CruiseMode)
					{
						if (moveVector.Z > 0)
						{
							CruiseMode = false;
						}
						else
						{
							moveVector.Z = -1;
						}
					}
				}

				public void Update()
				{
					ticker.Tick();
					gc.Update();
					gs.Update();
					// scanner.Scan(); // it might be heavy, so call it only when needed

					alignedVelocity2D.X = gs.AlignedVelocity.X;
					alignedVelocity2D.Y = gs.AlignedVelocity.Z;

					angularVelocity2D.X = gc.AngularVelocity.X;
					angularVelocity2D.Y = gc.AngularVelocity.Y;

					avgVelocity2D += (alignedVelocity2D - avgVelocity2D) * (alignedVelocity2D.Length() > avgVelocity2D.Length() ? .3f : .1f);
					currentMoveAngle = (float)Math.Atan2(gs.AlignedVelocity.Y, gs.AlignedVelocity.X);//check

					moveVector = currentController.MoveIndicator;
					ToggleCruiseMode();
					moveVector2D.X = moveVector.X;
					moveVector2D.Y = moveVector.Z;
					moveLenght = moveVector2D.Length();
					moveAngle = moveLenght > 0 ? (float)Math.Atan2(moveVector.X, moveVector.Z) : moveAngle;

					rotationIndidcator.X = currentController.RotationIndicator.X;
					rotationIndidcator.Y = currentController.RotationIndicator.Y;
					rotationIndidcator.Z = currentController.RollIndicator;

					pitch += rotationIndidcator.X * (float)ticker.dT * .05f;
					pitch = Math.Min(Math.Max(-maxPitch, pitch), maxPitch);
					pitch -= Math.Sign(pitch) * (float)ticker.dT * .05f;
					if (Math.Abs(pitch) < .005)
					{
						pitch = 0;
					}

					yaw += rotationIndidcator.Y * (float)ticker.dT;
					yaw *= .9f;
					if (Math.Abs(yaw) < .1)
					{
						yaw = 0;
					}

					currentHeight = idleHeight + idleHeight * Math.Sign(moveVector.Y) * .5f;
					offset = new Vector3(Math.Sin(gs.AvgGravity.Y) * currentHeight * 2 * 0, Math.Cos(gs.Gravity.Length()) * currentHeight, -Math.Sin(gs.AvgGravity.X) * currentHeight * 2 * 0);
				}
			}

			public class Ticker
			{
				public const double StepMin = MathHelper.Pi / 20;
				public const double StepMax = MathHelper.Pi / 6;

				public double Phase = 0;
				public double TickStep = StepMin;
				public double dT = 100d;
				public DateTime LastTime = DateTime.Now;

				public void Tick()
				{
					Phase = (Phase + TickStep * dT) % MathHelper.TwoPi;
					Update();
				}

				public void Update()
				{
					dT += ((DateTime.Now - LastTime).Milliseconds / 1000f - dT) * .1;
					LastTime = DateTime.Now;
				}

				readonly string[] visuals = new string[] { "|.........", "./........", "../.......", ".../......", "..../.....", "...../....", "....../...", "......./..", "......../.", ".........|", "........\\.", ".......\\..", "......\\...", ".....\\....", "....\\.....", "...\\......", "..\\.......", ".\\........" };
				int currentVisual = 0;
				public override string ToString()
				{
					return visuals[currentVisual++ % visuals.Length];
				}
			}

			public interface Actor
			{
				/// <returns>Next state</returns>
				Actor Do(VehicleData data);
			}

			public class IdleActor : Actor
			{
				public Actor Do(VehicleData data)
				{
					bool underControl = data.UnderControl;
					var error = data.scanner.SurfaceAnglesPitchRollRaw.Length() > 0 ? data.scanner.SurfaceAnglesPitchRoll : data.gs.AvgGravity;
					if (error.Length() < .1) error *= 0;
					data.currentController.HandBrake = true;
					data.gc.Apply(0, 0, 0, (float)data.ticker.dT, true, error);
					foreach (var wheel in data.wheels)
					{
						wheel.SetHeight(10);
						wheel.SetVelocity(0, 0);
					}
					return data.UnderControl ? new AWCActor() as Actor : this; ;
				}
			}

			public class AWCActor : Actor
			{
				public Actor Do(VehicleData data)
				{
					data.scanner.Scan();
					var expectedVelocity = data.gs.AlignedVelocity.Z * (data.moveVector.Z == 0 ? 0 : 1) - data.moveVector.Z * 10;
					if (Math.Sign(data.gs.AlignedVelocity.Z) == Math.Sign(data.moveVector.Z) || data.moveVector.Z == 0)
					{
						expectedVelocity = 0;
					}

					bool useTankSteering = Math.Abs(data.gs.AlignedVelocity.Z) < 3 && data.moveVector.Z == 0;
					data.currentController.HandBrake = expectedVelocity == 0 && !useTankSteering;

					var targetHeightMult = data.pitch * 10 * (useTankSteering && data.moveVector.X != 0 || Math.Abs(data.yaw) > 1 ? 0 : 1);// ignore pitch on yaw mousemove, otherwise wheel angle being reset sometimes
					foreach (var wheel in data.wheels)
					{
						wheel.SetHeight(-1.5f - wheel.PitchMult * targetHeightMult);
						if (useTankSteering)// tank steering
						{
							if (Math.Abs(data.yaw) > 1 || data.moveVector.X != 0)
							{
								wheel.SetSteer(-5 * wheel.AccermanSteerMult, true);
								var velocity = Math.Sign(data.yaw) * 5 * wheel.sideMult + Math.Sign(data.moveVector.X) * 5 * wheel.sideMult;
								wheel.SetVelocity(velocity, velocity);
							}
							else
							{
								wheel.SetSteer(0, true);
							}
						}
						else
						{
							wheel.SetVelocity(expectedVelocity, expectedVelocity);
							var mouseSteer = -data.yaw * wheel.AccermanSteerMult;
							wheel.SetSteer(data.moveVector.X + mouseSteer, false);
						}
					}

					var error = data.scanner.SurfaceAnglesPitchRollRaw.Length() > 0 ? data.scanner.SurfaceAnglesPitchRoll : data.gs.AvgGravity;
					if (error.Length() < .1) error *= 0;
					data.gc.Apply(0, 0, 0, (float)data.ticker.dT, true, error);

					return data.UnderControl || data.CruiseMode ? this : new IdleActor() as Actor;
				}
			}
		}

		public class SurfaceScanner
		{
			IMyCameraBlock camera;
			GridNode grid;

			public Vector3 SurfaceAnglesPitchRollRaw = new Vector3();// pitch yaw roll
			public Vector3 SurfaceAnglesPitchRoll { get { return lastUpdate > DateTime.Now.AddSeconds(-1) ? surfaceAnglesPitchRoll : new Vector3(); } }// don't provide outdated data
			public double roll;

			Vector3 surfaceAnglesPitchRoll = new Vector3();
			const float scanDistance = 50;
			const float scanConeAngle = 20;
			float lastScanAngle = 0;
			bool haveData = false;
			long[] subgridEntityIds;

			DateTime lastUpdate = new DateTime();
			TimeSpan scanInterval = TimeSpan.FromSeconds(.1);
			public SurfaceScanner(GridNode grid, List<IMyCameraBlock> cameras)
			{
				this.grid = grid;
				// cameras looking down
				this.camera = cameras.FirstOrDefault(c => grid.GetRelativeOrientation(c).Forward == grid.GetRelativeOrientation(grid.RootReference).Down);
				if (camera != null)
				{
					camera.EnableRaycast = true;
				}
				// exclude raycast collisions with subgrid
				subgridEntityIds = grid.AllGrids().Select(x => x.CubeGrid.EntityId).ToArray();
			}

			public void Scan()
			{
				if (camera == null || lastUpdate.Add(scanInterval) > DateTime.Now) return;
				var detected = camera.Raycast(scanDistance, (float)Math.Sin(lastScanAngle) * scanConeAngle, (float)Math.Cos(lastScanAngle) * scanConeAngle);
				lastScanAngle += MathHelper.Pi * .6f;

				if (detected.HitPosition.HasValue
					&& !subgridEntityIds.Contains(detected.EntityId)
					&& (detected.Type == MyDetectedEntityType.SmallGrid || detected.Type == MyDetectedEntityType.SmallGrid || detected.Type == MyDetectedEntityType.Planet || detected.Type == MyDetectedEntityType.Asteroid)
					)
				{
					AddSurfacePoint(detected.HitPosition.Value);
					validPoints = Math.Min(validPoints + 1, surfacePoints.Length);
				}
				else
				{
					validPoints = Math.Max(validPoints - 1, 0);
				}

				if (validPoints < surfacePoints.Length)
				{
					SurfaceAnglesPitchRollRaw *= 0;
					surfaceAnglesPitchRoll *= 0;
					return;// no points available to calculate pitch and roll
				}

				var cross = GetCross();

				var controllerMatrix = grid.RootReference.WorldMatrix;

				var referenceForward = controllerMatrix.Forward;
				var referenceLeft = controllerMatrix.Left;
				var referenceUp = controllerMatrix.Down;

				SurfaceAnglesPitchRollRaw.X = -(float)Math.Atan2(cross.Dot(referenceForward), cross.Dot(referenceUp));
				SurfaceAnglesPitchRollRaw.Z = (float)Math.Atan2(cross.Dot(referenceLeft), cross.Dot(referenceUp));

				surfaceAnglesPitchRoll += (SurfaceAnglesPitchRollRaw - SurfaceAnglesPitchRoll) * .1f;
			}

			int indexToUpdate;
			int validPoints;
			Vector3D[] surfacePoints = new Vector3D[3];

			protected void AddSurfacePoint(Vector3D point)
			{
				surfacePoints[indexToUpdate++ % surfacePoints.Length] = point;
				lastUpdate = DateTime.Now;
			}

			protected Vector3D GetCross()
			{
				var v1 = surfacePoints[(indexToUpdate + 1) % surfacePoints.Length] - surfacePoints[(indexToUpdate) % surfacePoints.Length];
				var v2 = surfacePoints[(indexToUpdate + 1) % surfacePoints.Length] - surfacePoints[(indexToUpdate + 2) % surfacePoints.Length];
				return v1.Cross(v2);
			}
		}

		public class GravityScanner
		{
			IMyShipController reference;
			public Vector3 Gravity = new Vector3();//pitch yaw roll
			public Vector3 AvgGravity = new Vector3();
			public float GravityLength = 0;
			public double Elevation = 0;

			public float CurrentDirection = 0;
			public float LastDirection = 0;
			public float DirectionDiff { get { return MathHelper.WrapAngle(CurrentDirection - LastDirection); } }

			public Vector3 AlignedVelocity = new Vector3();
			public Vector3 AlignedToGravityVelocity = new Vector3();

			public GravityScanner(IMyShipController reference)
			{
				this.reference = reference;
				Update();
				LastDirection = CurrentDirection;
			}

			public void Update()
			{
				reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out Elevation);

				var controllerMatrix = reference.WorldMatrix;
				var velocities = reference.GetShipVelocities();
				//gravity
				var gravityVec = reference.GetNaturalGravity();
				GravityLength = (float)gravityVec.Length();

				var referenceForward = controllerMatrix.Forward;
				var referenceLeft = controllerMatrix.Left;
				var referenceUp = controllerMatrix.Down;
				Gravity.X = -(float)Math.Atan2(gravityVec.Dot(referenceForward), gravityVec.Dot(referenceUp));
				Gravity.Z = (float)Math.Atan2(gravityVec.Dot(referenceLeft), gravityVec.Dot(referenceUp));
				AvgGravity += (Gravity - AvgGravity) * .2f;
				//aligned velocity
				var velocityVector = velocities.LinearVelocity;
				AlignedVelocity.X = (float)velocityVector.Dot(referenceLeft);
				AlignedVelocity.Y = (float)velocityVector.Dot(referenceUp);
				AlignedVelocity.Z = (float)velocityVector.Dot(referenceForward);
				//grav aligned velocity
				var matrix = Matrix.CreateFromYawPitchRoll(Gravity.X, -Gravity.Z, 0);//magic
				AlignedToGravityVelocity = Vector3.Transform(AlignedVelocity, matrix);

				var d = referenceForward.Cross(gravityVec);

				if (reference.RotationIndicator.Y != 0)
				{// update expected direction only if user turn rover with mouse
					LastDirection = CurrentDirection;
				}
				CurrentDirection = (float)Math.Atan2(d.X, d.Y);
			}
		}
		public class GyroController
		{
			List<KeyValuePair<Matrix, IMyGyro>> gyroMatrixes;
			List<KeyValuePair<Matrix, IMyGyro>> gyrosToControl;
			IMyShipController reference;
			const float turnTime = 5;//seconds

			public Vector3 AngularVelocity = new Vector3();
			public Vector3 AvgAngularVelocity = new Vector3();

			public Vector3 KillRotError = new Vector3();
			PIDController gravController = new PIDController { gainI = 0.0000f, gainP = 10.0f, gainD = 5f };

			public GyroController(IMyShipController reference, IEnumerable<IMyGyro> gyros)
			{
				this.reference = reference;
				gyroMatrixes = gyros.Select(g =>
				{
					Matrix shipMatrix;
					reference.Orientation.GetMatrix(out shipMatrix);
					Matrix gyroMatrix;
					g.Orientation.GetMatrix(out gyroMatrix);
					var res = shipMatrix * Matrix.Transpose(gyroMatrix);

					return new KeyValuePair<Matrix, IMyGyro>(res, g);
				}).ToList();
				ControlGyros();
			}

			public void Update()
			{
				var controllerMatrix = reference.WorldMatrix;
				var velocities = reference.GetShipVelocities();

				var referenceForward = controllerMatrix.Forward;
				var referenceLeft = controllerMatrix.Left;
				var referenceUp = controllerMatrix.Down;
				//angular velocity
				AngularVelocity = Vector3.Transform(velocities.AngularVelocity, MatrixD.Invert(controllerMatrix.GetOrientation()));
				AvgAngularVelocity += (AngularVelocity - AvgAngularVelocity) * .2f;
			}

			public void ControlGyros(float gyroPercent = 1)
			{
				var gyrosCountToControl = (int)(gyroMatrixes.Count * gyroPercent);
				gyrosToControl = gyroMatrixes.Take(gyrosCountToControl).ToList();
				foreach (var tuple in gyroMatrixes.Skip(gyrosCountToControl))
				{
					var gyro = tuple.Value;
					gyro.Pitch = gyro.Yaw = gyro.Roll = 0;
					gyro.GyroOverride = false;
				}
			}

			public void Apply(float pitch, float yaw, float roll, float dT, bool killRot = false, Vector3D? error = null)
			{
				var inputVec = new Vector3D(-pitch, yaw, roll) * dT;
				var rotationVec = new Vector3D();
				inputVec.X += reference.RotationIndicator.X * dT * 100;
				inputVec.Y += reference.RotationIndicator.Y * dT * 100;
				inputVec.Z += reference.RollIndicator * dT * 100;
				rotationVec += inputVec;
				if (killRot)
				{
					KillRotError += AngularVelocity * 200 * dT;
					if (KillRotError.Length() < .1) KillRotError *= 0;
					rotationVec += new Vector3D(
						MathHelperD.Clamp(KillRotError.X, -100, 100),
						MathHelperD.Clamp(KillRotError.Y, -100, 100),
						MathHelperD.Clamp(KillRotError.Z, -100, 100));
					KillRotError += inputVec;
					KillRotError *= 0.7f;
				}

				if (error.HasValue)
				{
					gravController.currentError.X = error.Value.X - pitch;
					gravController.currentError.Y = error.Value.Y - yaw;
					gravController.currentError.Z = error.Value.Z + roll;
					gravController.Step(dT);
					rotationVec += gravController.output;
				}

				bool applyOverride = inputVec.Length() > 0 || killRot && KillRotError.Length() != 0;

				rotationVec *= MathHelper.RadiansPerSecondToRPM;
				rotationVec.X = MathHelper.Clamp(rotationVec.X, -300, 300);
				rotationVec.Y = MathHelper.Clamp(rotationVec.Y, -300, 300);
				rotationVec.Z = MathHelper.Clamp(rotationVec.Z, -300, 300);
				foreach (var tuple in gyrosToControl)
				{
					var gyro = tuple.Value;
					if (applyOverride)
					{
						var rotation = Vector3.TransformNormal(rotationVec, tuple.Key);
						gyro.Pitch = rotation.X;
						gyro.Yaw = rotation.Y;
						gyro.Roll = rotation.Z;
					}
					gyro.GyroOverride = applyOverride;
				}

			}
			public void Stop()
			{
				var rotationVec = reference.GetShipVelocities().AngularVelocity;
				bool applyOverride = rotationVec.Length() > 0;
				foreach (var tuple in gyrosToControl)
				{
					var gyro = tuple.Value;
					if (applyOverride)
					{
						var rotation = Vector3.TransformNormal(rotationVec, tuple.Key) * MathHelper.RadiansPerSecondToRPM;
						gyro.Pitch = rotation.X;
						gyro.Yaw = rotation.Y;
						gyro.Roll = rotation.Z;
					}
					gyro.GyroOverride = applyOverride;
				}

			}

			public class PIDController
			{
				public float gainP = 1;
				public float gainI = 1;
				public float gainD = 1;

				public Vector3D currentError = new Vector3D();
				Vector3D previousError = new Vector3D();
				Vector3D integral = new Vector3D();
				public Vector3D output = new Vector3D();

				public void Step(float dt)
				{
					integral = dt * (integral + currentError) * .95;
					var derivative = (1 / dt) * (currentError - previousError);
					output = gainP * currentError + gainI * integral + gainD * derivative;

					previousError = currentError;
				}
			}
		}

		public class Wheel
		{
			public readonly IMyMotorSuspension wheel;
			public readonly GridNode gridNode;
			public readonly Vector3 RelativePosition;
			public readonly int sideMult = 0;
			public Vector3 WheelCenter { get; protected set; }
			public float AccermanSteerMult { get; protected set; }
			public float PitchMult { get; protected set; }

			public static IEnumerable<Wheel> GetWheels(GridNode grid, IEnumerable<IMyMotorSuspension> motors)
			{
				var wheels = motors.Where(c => grid.GetRelativeOrientation(c).Up == grid.GetRelativeOrientation(grid.RootReference).Right || grid.GetRelativeOrientation(c).Up == grid.GetRelativeOrientation(grid.RootReference).Left)
					.Select(x => new Wheel(x, grid)).ToList();
				var wheelCenter = new Vector3
				{
					X = wheels.Sum(x => x.RelativePosition.X) / wheels.Count,
					Y = wheels.Sum(x => x.RelativePosition.Y) / wheels.Count,
					Z = wheels.Sum(x => x.RelativePosition.Z) / wheels.Count,
				};

				var maxOffset = wheels.Max(w => Math.Abs(w.RelativePosition.Z - wheelCenter.Z));

				foreach (var wheel in wheels)
				{
					wheel.WheelCenter = wheelCenter;
					wheel.AccermanSteerMult = (wheel.RelativePosition.Z - wheelCenter.Z) / maxOffset;
					wheel.PitchMult = (wheel.RelativePosition.Z - wheelCenter.Z) / maxOffset;
				}

				return wheels;
			}

			public Wheel(IMyMotorSuspension wheel, GridNode gridNode)
			{
				this.wheel = wheel;
				this.gridNode = gridNode;
				RelativePosition = gridNode.GetRelativePosition(wheel);

				sideMult = RelativePosition.X > 0 ? -1 : 1;
			}

			public void SetVelocity(float velocity, float power)
			{
				velocity = MathHelper.Clamp(velocity, -1000, 1000);
				wheel.SetValue("Speed Limit", Math.Abs(velocity) * 3.6f);// m/s to km/h
				wheel.SetValue("Propulsion override", (float)Math.Atan2(power, 1) * sideMult);
			}

			public void SetSteer(float angle, bool useSideMult)
			{
				wheel.MaxSteerAngle = MathHelper.Clamp(Math.Abs(angle), 0, 1);
				var steer = MathHelper.Clamp(angle, -1, 1) * (useSideMult ? sideMult : 1);
				if (steer == 0) steer = 0.01f * sideMult;// don't return control to cockpit
				wheel.SetValue("Steer override", steer);
			}

			public void SetHeight(float height)
			{
				wheel.Height = height;
			}
		}

		public class GridNode
		{
			List<GridNode> ChildNodes = new List<GridNode>();
			public readonly GridNode ParentNode;

			public readonly IMyCubeGrid CubeGrid;
			readonly IMyCubeBlock Reference;

			readonly IMyMechanicalConnectionBlock ConnectionToParent;
			readonly IEnumerable<IMyMechanicalConnectionBlock> ConnectionsToParent;
			readonly IMyGridTerminalSystem gridTerminalSystem;
			public float ParentLength
			{
				get
				{
					if (ParentNode.ConnectionToParent == null) return 0;
					return ParentNode.ConnectionToParent.Top.Position.RectangularDistance(ConnectionToParent.Position) * ParentNode.CubeGrid.GridSize;
				}
			}
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
			IEnumerable<IMyMotorStator> _rotorsCW;
			public IEnumerable<IMyMotorStator> RotorsCW
			{
				get
				{
					if (_rotorsCW != null) return _rotorsCW;
					Matrix rootOrientation;
					RootReference.Orientation.GetMatrix(out rootOrientation);
					_rotorsCW = RotorsToParent.Where(x =>
					{
						var rotorOrientation = GetRelativeOrientation(x);
						return CwCriteria(rotorOrientation, rootOrientation);
					}).ToArray();
					return _rotorsCW;
				}
			}
			IEnumerable<IMyMotorStator> _rotorsCCW;
			public IEnumerable<IMyMotorStator> RotorsCCW
			{
				get
				{
					if (_rotorsCCW != null) return _rotorsCCW;
					Matrix rootOrientation;
					RootReference.Orientation.GetMatrix(out rootOrientation);
					_rotorsCCW = RotorsToParent.Where(x =>
					{
						var rotorOrientation = GetRelativeOrientation(x);
						return CcwCriteria(rotorOrientation, rootOrientation);
					}).ToArray();
					return _rotorsCCW;
				}
			}

			public GridNode(IMyGridTerminalSystem gridTerminalSystem, IMyCubeBlock reference, IEnumerable<IMyMechanicalConnectionBlock> allConnections)
			{
				if (reference == null) throw new ArgumentNullException("Reference cannot be null");
				this.gridTerminalSystem = gridTerminalSystem;
				this.CubeGrid = reference.CubeGrid;
				this.ConnectionsToParent = new List<IMyMechanicalConnectionBlock>();
				this.Reference = reference;
				ChildNodes = GetChildGrids(CubeGrid, allConnections).Select(x => new GridNode(x, allConnections, this)).ToList();
				Validate();
			}

			GridNode(IMyCubeGrid grid, IEnumerable<IMyMechanicalConnectionBlock> allConnections, GridNode parentNode)
			{
				this.CubeGrid = grid;
				this.ParentNode = parentNode;
				this.ConnectionsToParent = allConnections.Where(x => x.CubeGrid.EntityId == parentNode.CubeGrid.EntityId && x.TopGrid.EntityId == grid.EntityId).ToList();
				this.ConnectionToParent = ConnectionsToParent.FirstOrDefault();
				ChildNodes = GetChildGrids(CubeGrid, allConnections).Select(x => new GridNode(x, allConnections, this)).ToList();
			}

			public void Validate()//throw exception if something is wrong
			{
				var all = AllGrids().ToList();
				if (all.Any(g => g.RootReference == null))
				{
					throw new Exception("Some root reference is null");
				}
				all.ForEach(grid =>
				{
					if (grid.RotorsCW.Select(x => x.EntityId).Any(id => grid.RotorsCCW.Select(x => x.EntityId).Contains(id)))
					{//some rotors are in both lists
						throw new Exception($"Some rotors are in both CW and CCW\nCW\n{string.Join(",", grid.RotorsCW)}\nCCW\n{string.Join(",", grid.RotorsCCW)}" +
						$"CW\n{string.Join(",", grid.RotorsCW)}\nCCW\n{string.Join(",", grid.RotorsCCW)}");
					}
				});
			}

			public List<T> GetBlocksInGrid<T>()
				where T : class, IMyCubeBlock
			{
				var res = new List<T>();
				Root.gridTerminalSystem.GetBlocksOfType<T>(res, x => x.CubeGrid.EntityId == CubeGrid.EntityId);
				return res;
			}

			public IEnumerable<IMyMechanicalConnectionBlock> GetConnectionsToChild(GridNode node)
			{
				return ChildNodes.Where(x => x.CubeGrid.EntityId == node.CubeGrid.EntityId).Select(x => x.ConnectionToParent);
			}

			public GridNode Root
			{
				get
				{
					return ParentNode?.Root ?? this;
				}
			}

			public IMyCubeBlock RootReference
			{
				get
				{
					return Root.Reference;
				}
			}

			public Movement MovementToParent
			{
				get
				{
					if (PistonsToParent.Any()) return Movement.Extend;
					bool isRotors = RotorsToParent.Any();
					if (!isRotors) return Movement.Unknown;
					var orientation = ParentNode.GetRelativeOrientation(RotorsToParent.First());
					var rootOrientation = Root.GetRelativeOrientation(RootReference);
					if (YawCriteria(orientation, rootOrientation)) return Movement.Yaw;
					if (PitchCriteria(orientation, rootOrientation)) return Movement.Pitch;
					if (RollCriteria(orientation, rootOrientation)) return Movement.Roll;
					return Movement.Unknown;
				}
			}

			public Vector3 GetAveragePosition(IEnumerable<IMyCubeBlock> blocks)
			{
				var count = blocks.Count();
				var positions = blocks.Select(GetRelativePosition).ToList();
				return new Vector3
				{
					X = positions.Sum(p => p.X) / count,
					Y = positions.Sum(p => p.Y) / count,
					Z = positions.Sum(p => p.Z) / count,
				};
			}

			public Vector3 GetRelativePosition(Vector3 pos)
			{
				pos = TransformPosition(pos, ConnectionToParent, ParentNode?.Reference);
				return ParentNode?.GetRelativePosition(pos) ?? pos;
			}

			public Matrix GetRelativeOrientation(IMyCubeBlock block)
			{
				if (block == null) throw new Exception("Block is null");
				var grid = Root.GetGridForBlock(block);
				if (grid == null)
				{
					throw new Exception($"This block is not in child grid\n{block}\n{block.CubeGrid.CustomName}");
				}
				Matrix orientation;
				block.Orientation.GetMatrix(out orientation);

				if (grid.Reference != null)
				{
					//Echo($"Using reference {grid.Reference}");
					Matrix orientationReference;
					grid.Reference.Orientation.GetMatrix(out orientationReference);
					orientationReference = Matrix.Invert(orientationReference);
					orientation *= orientationReference;
					return orientation;
				}
				//Echo($"Transform orientation of {block}");
				return grid.GetRelativeOrientation(orientation);
			}
			public IEnumerable<Movement> MovementChain
			{
				get
				{
					if (ParentNode == null) return new List<Movement>();
					return (new[] { MovementToParent }).Concat(ParentNode.MovementChain);
				}
			}

			public IEnumerable<GridNode> PathToRoot
			{
				get
				{
					return (new List<GridNode>() { this }).Concat(ParentNode?.PathToRoot ?? new GridNode[0]);
				}
			}

			public IEnumerable<GridNode> AllGrids()
			{
				return (new List<GridNode>() { this }).Concat(ChildNodes.SelectMany(x => x.AllGrids()));
			}

			public GridNode GetGridForBlock(IMyCubeBlock block)
			{
				return AllGrids().Where(x => block.CubeGrid.EntityId == x.CubeGrid.EntityId).FirstOrDefault();
			}

			public IEnumerable<GridNode> GetGridChain(IEnumerable<Movement> expectedChain)
			{
				return AllGrids().Where(x =>
				{
					var chain = x.MovementChain;
					var chainEnumerator = chain.ToList().GetEnumerator();
					var expectedEnumerator = expectedChain.ToList().GetEnumerator();
					expectedEnumerator.MoveNext();
					//if (chain.FirstOrDefault() ==expectedEnumerator.Current) return false;

					//while (chainEnumerator.Current != expectedEnumerator.Current && chainEnumerator.MoveNext()) { };
					do
					{
						chainEnumerator.MoveNext();
						if (expectedEnumerator.Current != chainEnumerator.Current) return false;

					}
					while (expectedEnumerator.MoveNext());
					return true;
				}).ToList();
			}

			public Vector3 GetRelativePosition(IMyCubeBlock block)
			{
				if (block == null) throw new Exception("Block is null");
				var grid = Root.GetGridForBlock(block);
				if (grid == null)
				{
					throw new Exception($"This block is not in child grid\n{block}\n{block.CubeGrid.CustomName}");
				}
				if (grid.Reference != null)
				{
					return GetRelativePosition(block.Position, grid.Reference);
				}

				return grid.GetRelativePosition(block.Position);
			}

			public static Action<string> Echo = x => { };
			#region private
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
				return rotorOrientation.Up.Dot(rootOrientation.Backward) > .9;
			}
			static bool RollCcwCriteria(Matrix rotorOrientation, Matrix rootOrientation)
			{
				return rotorOrientation.Up.Dot(rootOrientation.Forward) > .9;
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

			static Vector3 GetRelativePosition(Vector3 pos, IMyCubeBlock reference)
			{
				var referencePos = new Vector3(reference.Position) * reference.CubeGrid.GridSize;
				var diff = pos - referencePos;
				Matrix rotation;
				reference.Orientation.GetMatrix(out rotation);
				rotation = Matrix.Invert(rotation);
				var res = Vector3.Transform(diff, rotation);
				return res;
			}

			Matrix GetRelativeOrientation(Matrix orientation)
			{
				orientation = TransformOrientation(orientation, ConnectionToParent, ParentNode?.Reference);
				return ParentNode?.GetRelativeOrientation(orientation) ?? orientation;
			}

			static IEnumerable<IMyCubeGrid> GetChildGrids(IMyCubeGrid grid, IEnumerable<IMyMechanicalConnectionBlock> connections)
			{
				return connections.Where(x => !(x is IMyMotorSuspension) && x.CubeGrid.EntityId == grid.EntityId).Select(x => x.TopGrid).Distinct();
			}

			static Vector3 TransformPosition(Vector3 src, IMyMechanicalConnectionBlock connection, IMyCubeBlock reference = null)
			{
				if (connection == null) return src;
				var top = connection.Top;
				var pos = GetRelativePosition(src, top);

				Matrix rotationConnection;
				connection.Orientation.GetMatrix(out rotationConnection);
				//rotation = Matrix.Invert(rotation);

				var translation = reference != null ? connection.Position - reference.Position : connection.Position;
				rotationConnection.Translation = translation;
				pos = Vector3.Transform(pos, rotationConnection);

				if (reference != null)
				{
					pos = AlignTo(pos, reference);
				}

				return pos;
			}
			static Matrix TransformOrientation(Matrix src, IMyMechanicalConnectionBlock connection, IMyCubeBlock reference = null)
			{
				//Echo($"\n\nTransform {src.Forward}");
				if (connection != null)
				{
					//Echo($"Using connection {connection}");
					var top = connection.Top;
					Matrix orientationTop;
					top.Orientation.GetMatrix(out orientationTop);
					orientationTop = Matrix.Invert(orientationTop);

					Matrix orientationConnection;
					connection.Orientation.GetMatrix(out orientationConnection);

					src *= orientationTop;
					src *= orientationConnection;
				}
				if (reference != null)
				{
					//Echo($"Using reference {reference}");
					Matrix orientationReference;
					reference.Orientation.GetMatrix(out orientationReference);
					orientationReference = Matrix.Invert(orientationReference);
					src *= orientationReference;
				}

				return src;
			}
			static Vector3 AlignTo(Vector3 pos, IMyCubeBlock reference)
			{
				Matrix rotation;
				reference.Orientation.GetMatrix(out rotation);
				rotation = Matrix.Invert(rotation);
				var res = Vector3.Transform(pos, rotation);
				return res;
			}
			#endregion private
			public override string ToString()
			{
				return $"Grid {CubeGrid.CustomName}\n ChildNodes:\n {string.Join("\n", ChildNodes.Select(x => x.ToString()))}";
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
#if DEBUG
	}
}
#endif
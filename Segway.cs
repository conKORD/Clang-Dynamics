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

namespace Segway
{
	public sealed class Program : MyGridProgram
	{
#endif
		Walker4 worker;
		IMyTextPanel debugScreen;
		Action<string> echo;
		Program()
		{
			debugScreen = GridTerminalSystem.GetBlockWithName("WDebug") as IMyTextPanel;
			echo = debugScreen != null ? t => { debugScreen.WritePublicText(t + "\n", true); Echo(t); } : Echo;
			try
			{
				worker = new Walker4(GridTerminalSystem, echo);
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
			debugScreen?.WritePublicText("");
			echo("Tick " + updateType);
			echo("Wheels  " + worker.walkerData.Wheels.Count);
			echo("State  " + worker.currentActor.ToString().Split('+').LastOrDefault());
			echo("TickTime  " + Math.Round(worker.walkerData.ticker.dT, 2));
			echo("TickStep  " + Math.Round(worker.walkerData.ticker.TickStep, 2));
			echo("Phase  " + Math.Round(worker.walkerData.ticker.Phase / Math.PI * 180, 2));
			echo($"Offset  {worker.walkerData.offset}");
			echo($"Height  {worker.walkerData.currentHeight}");
			try
			{
				worker.Tick(argument);
			}
			catch (Exception e)
			{
				Echo(e.Message);
				Echo(e.Source);
				Echo(e.StackTrace);
				Me.Enabled = false;
			}
		}

		public class Walker4
		{
			public static bool DEBUG = false;
			IMyGridTerminalSystem gridTerminalSystem;
			List<IMyShipController> controllers = new List<IMyShipController>();

			IMyShipController GetCurrentController()
			{
				return controllers.FirstOrDefault(x => x.CanControlShip && (x.IsUnderControl || ((x as IMyRemoteControl)?.IsAutoPilotEnabled ?? false)));
			}

			public WalkerData walkerData = new WalkerData();
			public Ticker ticker = new Ticker();

			List<string> debugMessages = new List<string>();
			IDictionary<string, Action> arguments;

			public static Action<string> Echo;

			public Actor currentActor;
			public IdleActor idleActor;

			public Walker4(IMyGridTerminalSystem gridTerminalSystem, Action<string> echo)
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

			public bool TryInit(WalkerData data)
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

				var connections = new List<IMyMechanicalConnectionBlock>();
				gridTerminalSystem.GetBlocksOfType(connections);
				data.gridNode = new GridNode(gridTerminalSystem, data.currentController, connections);

				var wheels = new List<IMyMotorSuspension>();
				gridTerminalSystem.GetBlocksOfType(wheels);

				data.Wheels = wheels.Select(w => new Wheel(w, data.gridNode)).ToList();

				data.wheelCenter = data.gridNode.GetAveragePosition(wheels);

				data.centerOfMass = data.gridNode.GetRelativePosition(data.currentController.CubeGrid.WorldToGridInteger(data.currentController.CenterOfMass));

				return true;
			}

			public class WalkerData
			{

				public bool CanWork { get { return currentController != null; } }
				public bool UnderControl { get { return currentController.IsUnderControl; } }


				public readonly float accelTime = 3f;//seconds
				public Ticker ticker = new Ticker();
				public GyroController gc;
				public IMyShipController currentController;
				public GridNode gridNode;

				public List<Wheel> Wheels = new List<Wheel>();

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

				public float maxPitch = MathHelper.PiOver4;
				public float pitch;
				public float yaw;

				public double stepSizeMult = 0;//decrease for strafe
				public double stepLength = 5;
				public double stepLengthMult = 0;
				public double stepHeight = 2;
				public Vector3 offset;


				public void Update()
				{
					ticker.Tick();
					gc.Update();

					alignedVelocity2D.X = gc.AlignedVelocity.X;
					alignedVelocity2D.Y = gc.AlignedVelocity.Z;

					angularVelocity2D.X = gc.AngularVelocity.X;
					angularVelocity2D.Y = gc.AngularVelocity.Y;

					avgVelocity2D += (alignedVelocity2D - avgVelocity2D) * (alignedVelocity2D.Length() > avgVelocity2D.Length() ? .3f : .1f);
					currentMoveAngle = (float)Math.Atan2(gc.AlignedVelocity.Y, gc.AlignedVelocity.X);//check

					moveVector = currentController.MoveIndicator;
					moveVector2D.X = moveVector.X;
					moveVector2D.Y = moveVector.Z;
					moveLenght = moveVector2D.Length();
					moveAngle = moveLenght > 0 ? (float)Math.Atan2(moveVector.X, moveVector.Z) : moveAngle;
					moveLenght -= moveLenght * Math.Abs(moveVector.X) * .2f;//decrease move length for strafe

					stepSizeMult = 1 - Math.Abs(Math.Sin(moveAngle)) * .8f;

					rotationIndidcator.X = currentController.RotationIndicator.X;
					rotationIndidcator.Y = currentController.RotationIndicator.Y;
					rotationIndidcator.Z = currentController.RollIndicator;

					pitch += rotationIndidcator.X / 100;
					pitch = Math.Min(Math.Max(-maxPitch, pitch), maxPitch);

					yaw += rotationIndidcator.Y / 100;
					yaw *= 0.99f;

					currentHeight = idleHeight + idleHeight * Math.Sign(moveVector.Y) * .5f;
					offset = new Vector3(Math.Sin(gc.AvgGravity.Y) * currentHeight * 2 * 0, Math.Cos(gc.Gravity.Length()) * currentHeight, -Math.Sin(gc.AvgGravity.X) * currentHeight * 2 * 0);
				}
			}

			public class Ticker
			{
				public const double StepIncrement = .001f;
				public const double StepMin = MathHelper.Pi / 20;
				public const double StepMax = MathHelper.Pi / 6;
				public const double AccelTime = 5d;

				const double landMult = .5;
				const double landPhase = MathHelper.TwoPi * (1 - landMult);
				const double airPhase = MathHelper.TwoPi * landMult;

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
			}

			public interface Actor
			{
				/// <returns>Next state</returns>
				Actor Do(WalkerData data);
			}

			public class IdleActor : Actor
			{
				public Actor Do(WalkerData data)
				{
					bool underControl = data.UnderControl;

					if (underControl && (data.moveLenght > .1 || data.avgVelocity2D.Length() >= 1 || Math.Abs(data.gc.AvgGravity.X) > .1)) return new PendulumSegwayActor();

					data.gc.Apply(0, 0, 0, (float)data.ticker.dT, true, true);
					foreach (var wheel in data.Wheels)
					{
						wheel.SetHeight(1);
						wheel.SetVelocity(0, 0);
					}
					return this;
				}
			}

			public class PendulumSegwayActor : Actor
			{

				static PIDController angleController = new PIDController();
				static float tilt = 0;
				public Actor Do(WalkerData data)
				{
					if (data.moveLenght < .1 && data.avgVelocity2D.Length() <= 1)
					{
						return new IdleActor();
					}
					tilt += (data.moveVector.Z - tilt) * (float)data.ticker.dT * 100;
					if (data.moveLenght < .1)
					{
						tilt = 0;
					}

					var targetAngle = tilt * .2f + (float)Math.Atan2(data.moveLenght > .1 ? 0 : data.gc.AlignedVelocity.Z, 50);
					var currentAngle = -data.gc.AvgGravity.X;


					var gravMult = (float)data.gc.GravityLength / Math.Abs(data.centerOfMassOffset.Y) * 2;
					var deltaVtoCompensateTilt = -(float)(Math.Sin(currentAngle + targetAngle) * gravMult);

					var deltaVtoCompensateAngularVelocity = Math.Cos(currentAngle) * data.gc.AvgAngularVelocity.X / data.centerOfMassOffset.Y;

					angleController.currentError = (targetAngle - currentAngle);
					angleController.Step((float)data.ticker.dT);
					var targetSpeed = data.gc.AlignedVelocity.Z + (float)Math.Atan2(angleController.output, 3) * 1000;

					foreach (var wheel in data.Wheels)
					{
						var power = angleController.output;
						wheel.SetHeight(-1);
						wheel.SetVelocity(27.777f, (float)power*.3f);
					}
					data.gc.Apply(targetAngle * 1.3f, 0, 0, (float)data.ticker.dT, true, true);
					return this;
				}

				public class PIDController
				{
					public float gainP = 2;
					public float gainI = .01f;
					public float gainD = .3f;

					public float currentError;
					float previousError;
					float integral;
					public float output;

					public void Step(float dt)
					{
						integral = MathHelper.Clamp(dt * (integral + currentError) * .95f, -1e5f, 1e5f);
						var derivative = (1 / dt) * (currentError - previousError);
						output = gainP * currentError + gainI * integral + gainD * derivative;

						previousError = currentError;
					}
				}
			}
		}

		public struct LegMove
		{
			public Vector3D StepPos;
			public Vector3D OffsetPos;
			public Vector3D FeetPitchYawRoll;
			public float currentPhase;
			public bool contactGround;
			public bool forceOffset;
			public float commitTime;
		}

		public class Wheel
		{
			public readonly IMyMotorSuspension wheel;
			public readonly GridNode gridNode;
			public readonly Vector3 relativePosition;
			public readonly int sideMult = 0;

			public Wheel(IMyMotorSuspension wheel, GridNode gridNode)
			{
				this.wheel = wheel;
				this.gridNode = gridNode;
				relativePosition = gridNode.GetRelativePosition(wheel);

				sideMult = relativePosition.X > 0 ? -1 : 1;
			}

			public void SetVelocity(float velocity, float power)
			{
				velocity = MathHelper.Clamp(velocity, -1000, 1000);
				wheel.SetValue("Speed Limit", Math.Abs(velocity) * 3.6f);// m/s to km/h
				wheel.SetValue("Propulsion override", (float)Math.Atan2(power, 1) * sideMult);

			}
			public void SetHeight(float height)
			{
				wheel.Height = height;
			}
		}

		public class GyroController
		{
			List<KeyValuePair<Matrix, IMyGyro>> gyroMatrixes;
			List<KeyValuePair<Matrix, IMyGyro>> gyrosToControl;
			IMyShipController reference;
			const float turnTime = 5;//seconds

			public Vector2 Gravity = new Vector2();//pitch roll
			public Vector2 AvgGravity = new Vector2();
			public float GravityLength = 0;
			public double Elevation = 0;

			public Vector3 AlignedVelocity = new Vector3();
			public Vector3 AlignedToGravityVelocity = new Vector3();
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
				reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out Elevation);

				var controllerMatrix = reference.WorldMatrix;
				var velocities = reference.GetShipVelocities();
				//gravity
				var gravityVec = reference.GetNaturalGravity();
				GravityLength = (float)gravityVec.Length();

				var referenceForward = controllerMatrix.Forward;
				var referenceLeft = controllerMatrix.Left;
				var referenceUp = controllerMatrix.Down;
				Gravity.X = (float)Math.Atan2(gravityVec.Dot(referenceForward), gravityVec.Dot(referenceUp));
				Gravity.Y = (float)Math.Atan2(gravityVec.Dot(referenceLeft), gravityVec.Dot(referenceUp));
				AvgGravity += (Gravity - AvgGravity) * .2f;
				//aligned velocity
				var velocityVector = velocities.LinearVelocity;
				AlignedVelocity.X = (float)velocityVector.Dot(referenceLeft);
				AlignedVelocity.Y = (float)velocityVector.Dot(referenceUp);
				AlignedVelocity.Z = (float)velocityVector.Dot(referenceForward);
				//grav aligned velocity
				var matrix = Matrix.CreateFromYawPitchRoll(Gravity.X, -Gravity.Y, 0);//magic
				AlignedToGravityVelocity = Vector3.Transform(AlignedVelocity, matrix);
				//angular velocity
				AngularVelocity = Vector3.Transform(velocities.AngularVelocity, MatrixD.Invert(controllerMatrix.GetOrientation()));
				AvgAngularVelocity += (AngularVelocity - AvgAngularVelocity) * .2f;
			}

			public void ControlGyros(float gyroPercent = 1)
			{
				var gyrosCouuntToControl = (int)(gyroMatrixes.Count * gyroPercent);
				gyrosToControl = gyroMatrixes.Take(gyrosCouuntToControl).ToList();
				foreach (var tuple in gyroMatrixes.Skip(gyrosCouuntToControl))
				{
					var gyro = tuple.Value;
					gyro.Pitch = gyro.Yaw = gyro.Roll = 0;
					gyro.GyroOverride = false;
				}
			}

			public void Apply(float pitch, float yaw, float roll, float dT, bool killRot = false, bool alignToGravity = false)
			{
				var inputVec = new Vector3D(-pitch, yaw, roll) * dT;
				var rotationVec = new Vector3D();
				bool applyOverride = rotationVec.Length() > 0 || killRot;
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
				if (alignToGravity)
				{
					gravController.currentError.X = -Gravity.X - pitch;
					gravController.currentError.Z = Gravity.Y + roll;
					gravController.Step(dT);
					rotationVec += gravController.output;
				}
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
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

namespace EmptyProgram
{
	public sealed class Program : MyGridProgram
	{
#endif
		IMyTextPanel debugScreen;
		Worker worker;
		Action<string> echo;

		Program()
		{
			debugScreen = GridTerminalSystem.GetBlockWithName("WDebug") as IMyTextPanel;
			echo = debugScreen != null ? t => { debugScreen.WritePublicText(t + "\n", true); Echo(t); } : Echo;
			try
			{
				worker = new Worker(GridTerminalSystem, echo);
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

		public class Worker
		{
			private IMyGridTerminalSystem gridTerminalSystem;
			private Action<string> echo;

			public Worker(IMyGridTerminalSystem gridTerminalSystem, Action<string> echo)
			{
				this.gridTerminalSystem = gridTerminalSystem;
				this.echo = echo;
			}

			public void Tick(string argument)
			{
				echo("Hello world!");
			}
		}
#if DEBUG
	}
}
#endif
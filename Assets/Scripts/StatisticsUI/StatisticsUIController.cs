using UnityEngine;

namespace Labust.StatisticsUI
{
	public class StatisticsUIController : MonoBehaviour
	{
		private Canvas _canvas;
		private PathRecordingsVisualization _controller;
		
		void Start()
		{
			_canvas = GetComponent<Canvas>();
			_canvas.enabled = false;
			_controller = GetComponentInChildren<PathRecordingsVisualization>();
		}

		void Update()
		{
			if (Input.GetKeyDown(KeyCode.P))
			{
				if (_canvas.enabled)
				{
					_canvas.enabled = false;
				}
				else
				{
					_canvas.enabled = true;
					_controller.RefreshPaths();
				}
			}
		}
	}
}

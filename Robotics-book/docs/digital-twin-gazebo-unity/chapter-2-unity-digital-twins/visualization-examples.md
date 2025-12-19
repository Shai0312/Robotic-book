---
title: Unity Visualization Examples
sidebar_label: Visualization Examples
sidebar_position: 6
description: Practical examples of visualization techniques for digital twins in Unity
---

# Unity Visualization Examples

This section provides practical examples of visualization techniques for digital twins in Unity. These examples demonstrate how to create effective visual representations of robots and their environments, with a focus on Human-Robot Interaction (HRI) applications.

## Basic Robot Visualization

### Simple Robot Model Setup

Let's start with a basic robot visualization example that demonstrates the core components:

```csharp
using UnityEngine;

public class BasicRobotVisualizer : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "Robot1";
    public Color robotColor = Color.gray;

    [Header("Joint Visualization")]
    public Transform[] joints;
    public GameObject jointPrefab;

    [Header("Status Indicators")]
    public Light statusLight;
    public Renderer robotRenderer;

    void Start()
    {
        InitializeRobot();
    }

    void InitializeRobot()
    {
        // Set robot color
        if (robotRenderer != null)
        {
            robotRenderer.material.color = robotColor;
        }

        // Initialize status light
        if (statusLight != null)
        {
            statusLight.color = Color.green; // Default to operational
        }

        // Create joint visualizations if needed
        if (joints.Length > 0 && jointPrefab != null)
        {
            CreateJointVisualizations();
        }
    }

    void CreateJointVisualizations()
    {
        foreach (Transform joint in joints)
        {
            GameObject jointViz = Instantiate(jointPrefab, joint.position, joint.rotation);
            jointViz.transform.SetParent(joint);
            jointViz.name = joint.name + "_Viz";
        }
    }

    public void SetStatusOperational()
    {
        if (statusLight != null)
        {
            statusLight.color = Color.green;
        }
    }

    public void SetStatusWarning()
    {
        if (statusLight != null)
        {
            statusLight.color = Color.yellow;
        }
    }

    public void SetStatusError()
    {
        if (statusLight != null)
        {
            statusLight.color = Color.red;
            statusLight.intensity = 2f; // Make it more noticeable
        }
    }
}
```

### Robot State Visualization

Create a more advanced state visualization system:

```csharp
using UnityEngine;
using System.Collections.Generic;

public enum RobotState
{
    IDLE,
    MOVING,
    WORKING,
    ERROR,
    CHARGING
}

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("State Visualization")]
    public RobotState currentState = RobotState.IDLE;
    public Material idleMaterial;
    public Material movingMaterial;
    public Material workingMaterial;
    public Material errorMaterial;
    public Material chargingMaterial;

    [Header("Animation Controls")]
    public Animator robotAnimator;
    public bool useAnimation = true;

    [Header("Particle Effects")]
    public ParticleSystem workingParticles;
    public ParticleSystem chargingParticles;

    [Header("Audio Feedback")]
    public AudioSource audioSource;
    public AudioClip idleSound;
    public AudioClip movingSound;
    public AudioClip workingSound;
    public AudioClip errorSound;

    private Dictionary<RobotState, Material> stateMaterials;
    private Dictionary<RobotState, AudioClip> stateSounds;

    void Start()
    {
        InitializeStateVisualization();
    }

    void InitializeStateVisualization()
    {
        stateMaterials = new Dictionary<RobotState, Material>
        {
            { RobotState.IDLE, idleMaterial },
            { RobotState.MOVING, movingMaterial },
            { RobotState.WORKING, workingMaterial },
            { RobotState.ERROR, errorMaterial },
            { RobotState.CHARGING, chargingMaterial }
        };

        stateSounds = new Dictionary<RobotState, AudioClip>
        {
            { RobotState.IDLE, idleSound },
            { RobotState.MOVING, movingSound },
            { RobotState.WORKING, workingSound },
            { RobotState.ERROR, errorSound }
        };

        UpdateVisualization();
    }

    void Update()
    {
        // Update based on current state
        UpdateVisualization();
    }

    public void SetRobotState(RobotState newState)
    {
        if (currentState != newState)
        {
            currentState = newState;
            UpdateVisualization();
        }
    }

    void UpdateVisualization()
    {
        // Update material based on state
        if (stateMaterials.ContainsKey(currentState))
        {
            Renderer robotRenderer = GetComponent<Renderer>();
            if (robotRenderer != null)
            {
                robotRenderer.material = stateMaterials[currentState];
            }
        }

        // Update animations
        if (robotAnimator != null && useAnimation)
        {
            robotAnimator.SetInteger("State", (int)currentState);
        }

        // Update particle effects
        UpdateParticleEffects();

        // Play appropriate sound
        PlayStateSound();
    }

    void UpdateParticleEffects()
    {
        if (workingParticles != null)
        {
            workingParticles.gameObject.SetActive(currentState == RobotState.WORKING);
        }

        if (chargingParticles != null)
        {
            chargingParticles.gameObject.SetActive(currentState == RobotState.CHARGING);
        }
    }

    void PlayStateSound()
    {
        if (audioSource != null && stateSounds.ContainsKey(currentState))
        {
            if (audioSource.clip != stateSounds[currentState])
            {
                audioSource.clip = stateSounds[currentState];
                audioSource.Play();
            }
        }
    }

    // Helper methods for state transitions
    public void StartMoving()
    {
        SetRobotState(RobotState.MOVING);
    }

    public void StartWorking()
    {
        SetRobotState(RobotState.WORKING);
    }

    public void StartCharging()
    {
        SetRobotState(RobotState.CHARGING);
    }

    public void SetError()
    {
        SetRobotState(RobotState.ERROR);
    }

    public void SetIdle()
    {
        SetRobotState(RobotState.IDLE);
    }
}
```

## Sensor Data Visualization

### LIDAR Point Cloud Visualization

Visualize LIDAR data as a point cloud in Unity:

```csharp
using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class LIDARVisualization : MonoBehaviour
{
    [Header("LIDAR Configuration")]
    public int numberOfRays = 360;
    public float maxRange = 10f;
    public float pointSize = 0.05f;

    [Header("Visualization")]
    public Color pointColor = Color.blue;
    public bool useRealtimeData = true;

    private Mesh pointCloudMesh;
    private Vector3[] points;
    private Color[] colors;
    private int[] triangles;

    void Start()
    {
        InitializePointCloud();
    }

    void InitializePointCloud()
    {
        points = new Vector3[numberOfRays];
        colors = new Color[numberOfRays];
        triangles = new int[numberOfRays * 3];

        for (int i = 0; i < numberOfRays; i++)
        {
            points[i] = Vector3.zero;
            colors[i] = pointColor;
        }

        CreateMesh();
    }

    void CreateMesh()
    {
        pointCloudMesh = new Mesh();
        GetComponent<MeshFilter>().mesh = pointCloudMesh;

        UpdateMesh();
    }

    void UpdateMesh()
    {
        pointCloudMesh.Clear();
        pointCloudMesh.vertices = points;
        pointCloudMesh.colors = colors;

        // Create triangles to represent points (using small quads)
        for (int i = 0; i < numberOfRays; i++)
        {
            int triangleIndex = i * 6; // 6 indices per quad
            // This is a simplified approach - for true point cloud, you might want to use a different approach
            // such as instancing or a custom shader
        }

        pointCloudMesh.RecalculateBounds();
    }

    public void UpdateLIDARData(float[] ranges)
    {
        if (ranges.Length != numberOfRays)
        {
            Debug.LogWarning("LIDAR data length doesn't match expected number of rays");
            return;
        }

        for (int i = 0; i < numberOfRays; i++)
        {
            float angle = (2 * Mathf.PI * i) / numberOfRays;
            float distance = Mathf.Clamp(ranges[i], 0, maxRange);

            points[i] = new Vector3(
                distance * Mathf.Cos(angle),
                0,
                distance * Mathf.Sin(angle)
            );

            // Color based on distance
            float normalizedDistance = distance / maxRange;
            colors[i] = Color.Lerp(Color.red, Color.blue, normalizedDistance);
        }

        UpdateMesh();
    }

    // For demonstration purposes - generates fake LIDAR data
    public void GenerateFakeData()
    {
        float[] fakeData = new float[numberOfRays];
        for (int i = 0; i < numberOfRays; i++)
        {
            // Create a simple pattern with some obstacles
            float angle = (2 * Mathf.PI * i) / numberOfRays;
            float distance = maxRange * 0.7f; // Base distance

            // Add some obstacles
            if (i > 80 && i < 100) distance = 3f; // Wall at front
            if (i > 260 && i < 280) distance = 2f; // Wall at back

            fakeData[i] = distance;
        }

        UpdateLIDARData(fakeData);
    }
}
```

### Camera Feed Visualization

Display camera feeds from the digital twin:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class CameraFeedVisualizer : MonoBehaviour
{
    [Header("Camera Configuration")]
    public string cameraTopic = "/camera/image_raw";
    public int width = 640;
    public int height = 480;

    [Header("Visualization")]
    public RawImage displayImage;
    public bool showAsOverlay = false;
    public float overlayOpacity = 0.7f;

    [Header("Processing Options")]
    public bool showDepthOverlay = false;
    public Color depthColor = Color.red;

    private Texture2D cameraTexture;
    private Color32[] pixelData;

    void Start()
    {
        InitializeCameraFeed();
    }

    void InitializeCameraFeed()
    {
        // Create texture for camera feed
        cameraTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        pixelData = new Color32[width * height];

        if (displayImage != null)
        {
            displayImage.texture = cameraTexture;
        }

        // Initialize with placeholder data
        InitializePlaceholderData();
    }

    void InitializePlaceholderData()
    {
        for (int i = 0; i < pixelData.Length; i++)
        {
            pixelData[i] = Color.gray;
        }
        UpdateTexture();
    }

    public void UpdateCameraFeed(Color32[] newPixelData)
    {
        if (newPixelData.Length != pixelData.Length)
        {
            Debug.LogWarning("Camera data size mismatch");
            return;
        }

        pixelData = newPixelData;
        UpdateTexture();
    }

    void UpdateTexture()
    {
        cameraTexture.SetPixels32(pixelData);
        cameraTexture.Apply();
    }

    // Method to add depth overlay to camera feed
    public void AddDepthOverlay(float[] depthData)
    {
        if (!showDepthOverlay || depthData.Length != width * height)
        {
            return;
        }

        Color32[] overlayData = new Color32[pixelData.Length];

        for (int i = 0; i < pixelData.Length; i++)
        {
            float depthValue = depthData[i];
            if (depthValue > 0 && depthValue < 5.0f) // Only highlight close objects
            {
                float intensity = Mathf.Clamp01(depthValue / 5.0f);
                Color overlay = Color.Lerp(Color.clear, depthColor, 1 - intensity);
                overlayData[i] = Color32.Lerp(pixelData[i], (Color32)overlay, overlay.a);
            }
            else
            {
                overlayData[i] = pixelData[i];
            }
        }

        cameraTexture.SetPixels32(overlayData);
        cameraTexture.Apply();
    }

    // Helper method to convert byte array to Color32 array
    public Color32[] ConvertByteArrayToColor32(byte[] data)
    {
        Color32[] colors = new Color32[data.Length / 3]; // Assuming RGB format

        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = new Color32(
                data[i * 3],     // R
                data[i * 3 + 1], // G
                data[i * 3 + 2], // B
                255              // A
            );
        }

        return colors;
    }
}
```

## Environment Visualization

### Occupancy Grid Visualization

Visualize occupancy grids for navigation:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class OccupancyGridVisualizer : MonoBehaviour
{
    [Header("Grid Configuration")]
    public int gridWidth = 100;
    public int gridHeight = 100;
    public float cellSize = 0.1f;

    [Header("Visualization")]
    public Color freeSpaceColor = Color.green;
    public Color occupiedColor = Color.red;
    public Color unknownColor = Color.gray;
    public bool showGridLines = true;

    [Header("Display Options")]
    public bool autoUpdate = true;
    public float updateInterval = 0.5f;

    private Texture2D gridTexture;
    private float[,] occupancyData;
    private float lastUpdateTime;

    void Start()
    {
        InitializeGrid();
    }

    void InitializeGrid()
    {
        occupancyData = new float[gridWidth, gridHeight];
        gridTexture = new Texture2D(gridWidth, gridHeight, TextureFormat.RGB24, false);
        gridTexture.filterMode = FilterMode.Point; // Prevent interpolation

        // Initialize with unknown values
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                occupancyData[x, y] = -1; // Unknown
            }
        }

        UpdateGridTexture();
    }

    void Update()
    {
        if (autoUpdate && Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateGridTexture();
            lastUpdateTime = Time.time;
        }
    }

    public void UpdateOccupancyData(float[,] newData)
    {
        if (newData.GetLength(0) != gridWidth || newData.GetLength(1) != gridHeight)
        {
            Debug.LogWarning("Occupancy grid size mismatch");
            return;
        }

        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                occupancyData[x, y] = newData[x, y];
            }
        }
    }

    void UpdateGridTexture()
    {
        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                Color cellColor = GetCellColor(occupancyData[x, y]);
                gridTexture.SetPixel(x, y, cellColor);
            }
        }

        gridTexture.Apply();
    }

    Color GetCellColor(float occupancyValue)
    {
        if (occupancyValue < 0) return unknownColor; // Unknown
        if (occupancyValue < 0.2f) return freeSpaceColor; // Free space
        if (occupancyValue > 0.8f) return occupiedColor; // Occupied

        // Interpolate between free and occupied for uncertain areas
        float certainty = occupancyValue;
        return Color.Lerp(freeSpaceColor, occupiedColor, certainty);
    }

    // Helper method to convert to world coordinates
    public Vector3 GridToWorld(int gridX, int gridY)
    {
        float worldX = (gridX - gridWidth / 2) * cellSize;
        float worldY = (gridY - gridHeight / 2) * cellSize;
        return new Vector3(worldX, 0, worldY);
    }

    // Helper method to convert from world coordinates
    public Vector2Int WorldToGrid(Vector3 worldPos)
    {
        int gridX = Mathf.RoundToInt(worldPos.x / cellSize + gridWidth / 2);
        int gridY = Mathf.RoundToInt(worldPos.z / cellSize + gridHeight / 2);

        // Clamp to grid bounds
        gridX = Mathf.Clamp(gridX, 0, gridWidth - 1);
        gridY = Mathf.Clamp(gridY, 0, gridHeight - 1);

        return new Vector2Int(gridX, gridY);
    }
}
```

## Path Planning Visualization

### Navigation Path Visualization

Visualize planned and executed paths:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class PathVisualization : MonoBehaviour
{
    [Header("Path Configuration")]
    public List<Vector3> plannedPath = new List<Vector3>();
    public List<Vector3> executedPath = new List<Vector3>();

    [Header("Visualization")]
    public Color plannedPathColor = Color.blue;
    public Color executedPathColor = Color.green;
    public Color currentGoalColor = Color.red;
    public float pathLineWidth = 0.1f;

    [Header("Markers")]
    public GameObject pathMarkerPrefab;
    public float markerSpacing = 1.0f;

    [Header("Real-time Updates")]
    public bool autoUpdate = true;
    public int maxPathPoints = 1000;

    private LineRenderer plannedPathRenderer;
    private LineRenderer executedPathRenderer;
    private GameObject currentGoalMarker;

    void Start()
    {
        InitializePathVisualization();
    }

    void InitializePathVisualization()
    {
        // Create LineRenderers for paths
        plannedPathRenderer = CreateLineRenderer("PlannedPath", plannedPathColor);
        executedPathRenderer = CreateLineRenderer("ExecutedPath", executedPathColor);
    }

    LineRenderer CreateLineRenderer(string name, Color color)
    {
        GameObject lineObj = new GameObject(name);
        LineRenderer lineRenderer = lineObj.AddComponent<LineRenderer>();

        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = pathLineWidth;
        lineRenderer.endWidth = pathLineWidth;
        lineRenderer.positionCount = 0;

        lineRenderer.useWorldSpace = true;
        lineRenderer.transform.SetParent(transform);

        return lineRenderer;
    }

    public void UpdatePlannedPath(List<Vector3> newPath)
    {
        plannedPath = new List<Vector3>(newPath);
        UpdatePlannedPathRenderer();
    }

    public void UpdateExecutedPath(List<Vector3> newPath)
    {
        executedPath = new List<Vector3>(newPath);
        UpdateExecutedPathRenderer();
    }

    void UpdatePlannedPathRenderer()
    {
        if (plannedPathRenderer != null && plannedPath.Count > 1)
        {
            plannedPathRenderer.positionCount = plannedPath.Count;
            plannedPathRenderer.SetPositions(plannedPath.ToArray());
        }
    }

    void UpdateExecutedPathRenderer()
    {
        if (executedPathRenderer != null && executedPath.Count > 1)
        {
            executedPathRenderer.positionCount = executedPath.Count;
            executedPathRenderer.SetPositions(executedPath.ToArray());
        }
    }

    public void AddPlannedPathPoint(Vector3 point)
    {
        if (plannedPath.Count >= maxPathPoints)
        {
            plannedPath.RemoveAt(0); // Remove oldest point
        }
        plannedPath.Add(point);
        UpdatePlannedPathRenderer();
    }

    public void AddExecutedPathPoint(Vector3 point)
    {
        if (executedPath.Count >= maxPathPoints)
        {
            executedPath.RemoveAt(0); // Remove oldest point
        }
        executedPath.Add(point);
        UpdateExecutedPathRenderer();
    }

    public void ClearPaths()
    {
        plannedPath.Clear();
        executedPath.Clear();

        if (plannedPathRenderer != null) plannedPathRenderer.positionCount = 0;
        if (executedPathRenderer != null) executedPathRenderer.positionCount = 0;
    }

    public void SetCurrentGoal(Vector3 goalPosition)
    {
        // Remove existing marker
        if (currentGoalMarker != null)
        {
            Destroy(currentGoalMarker);
        }

        // Create new marker
        if (pathMarkerPrefab != null)
        {
            currentGoalMarker = Instantiate(pathMarkerPrefab, goalPosition, Quaternion.identity);
            currentGoalMarker.GetComponent<Renderer>().material.color = currentGoalColor;
        }
        else
        {
            // Create simple sphere marker if no prefab is provided
            currentGoalMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            currentGoalMarker.transform.position = goalPosition;
            currentGoalMarker.transform.localScale = Vector3.one * 0.2f;
            currentGoalMarker.GetComponent<Renderer>().material.color = currentGoalColor;
            Destroy(currentGoalMarker.GetComponent<SphereCollider>()); // Remove collider
        }
    }

    // Method to visualize path comparison
    public void VisualizePathDeviation()
    {
        if (plannedPath.Count == 0 || executedPath.Count == 0)
            return;

        // Visualize deviation between planned and executed paths
        for (int i = 0; i < Mathf.Min(plannedPath.Count, executedPath.Count); i++)
        {
            Vector3 deviation = executedPath[i] - plannedPath[i];
            float distance = deviation.magnitude;

            if (distance > 0.1f) // Only show significant deviations
            {
                // Create deviation indicator
                GameObject deviationIndicator = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                deviationIndicator.transform.position = (plannedPath[i] + executedPath[i]) / 2;
                deviationIndicator.transform.LookAt(executedPath[i]);
                deviationIndicator.transform.localScale = new Vector3(0.05f, distance / 2, 0.05f);
                deviationIndicator.GetComponent<Renderer>().material.color = Color.Lerp(Color.yellow, Color.red, distance);

                Destroy(deviationIndicator, 5f); // Remove after 5 seconds
            }
        }
    }
}
```

## Interactive Control Visualization

### Control Interface Visualization

Create interactive controls for the digital twin:

```csharp
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class ControlInterfaceVisualizer : MonoBehaviour
{
    [Header("Control Configuration")]
    public Transform robotTransform;
    public float moveSpeed = 1.0f;
    public float rotateSpeed = 90.0f;

    [Header("UI Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button rotateLeftButton;
    public Button rotateRightButton;
    public Text statusText;

    [Header("Visualization")]
    public GameObject velocityVector;
    public GameObject directionIndicator;
    public bool showVelocityVector = true;

    [Header("Safety Limits")]
    public float maxLinearVelocity = 2.0f;
    public float maxAngularVelocity = 90.0f;

    private float currentLinearVelocity = 0f;
    private float currentAngularVelocity = 0f;
    private bool isMoving = false;

    void Start()
    {
        InitializeControlInterface();
    }

    void InitializeControlInterface()
    {
        // Set up slider events
        if (linearVelocitySlider != null)
        {
            linearVelocitySlider.minValue = -maxLinearVelocity;
            linearVelocitySlider.maxValue = maxLinearVelocity;
            linearVelocitySlider.onValueChanged.AddListener(OnLinearVelocityChanged);
        }

        if (angularVelocitySlider != null)
        {
            angularVelocitySlider.minValue = -maxAngularVelocity;
            angularVelocitySlider.maxValue = maxAngularVelocity;
            angularVelocitySlider.onValueChanged.AddListener(OnAngularVelocityChanged);
        }

        // Set up button events
        SetupButtonEvents();

        // Initialize visualization objects
        InitializeVisualization();
    }

    void SetupButtonEvents()
    {
        if (moveForwardButton != null)
        {
            moveForwardButton.onClick.AddListener(() => SetLinearVelocity(maxLinearVelocity));
        }

        if (moveBackwardButton != null)
        {
            moveBackwardButton.onClick.AddListener(() => SetLinearVelocity(-maxLinearVelocity));
        }

        if (rotateLeftButton != null)
        {
            rotateLeftButton.onClick.AddListener(() => SetAngularVelocity(maxAngularVelocity));
        }

        if (rotateRightButton != null)
        {
            rotateRightButton.onClick.AddListener(() => SetAngularVelocity(-maxAngularVelocity));
        }
    }

    void InitializeVisualization()
    {
        if (velocityVector != null)
        {
            velocityVector.SetActive(showVelocityVector);
        }

        if (directionIndicator != null)
        {
            directionIndicator.SetActive(true);
        }
    }

    void Update()
    {
        UpdateRobotMovement();
        UpdateVisualization();
        UpdateStatus();
    }

    void UpdateRobotMovement()
    {
        if (robotTransform != null)
        {
            // Apply linear movement
            Vector3 forwardMovement = robotTransform.forward * currentLinearVelocity * Time.deltaTime;
            robotTransform.position += forwardMovement;

            // Apply angular movement
            Vector3 rotation = Vector3.up * currentAngularVelocity * Time.deltaTime;
            robotTransform.Rotate(rotation);

            isMoving = Mathf.Abs(currentLinearVelocity) > 0.01f || Mathf.Abs(currentAngularVelocity) > 0.01f;
        }
    }

    void UpdateVisualization()
    {
        if (velocityVector != null && showVelocityVector)
        {
            // Update velocity vector visualization
            Vector3 velocityDirection = robotTransform.forward * currentLinearVelocity;
            velocityVector.transform.position = robotTransform.position + Vector3.up * 0.5f;
            velocityVector.transform.rotation = Quaternion.LookRotation(velocityDirection, Vector3.up);
            velocityVector.transform.localScale = new Vector3(
                0.1f,
                0.1f,
                Mathf.Abs(currentLinearVelocity) * 2f
            );
        }

        if (directionIndicator != null)
        {
            // Update direction indicator
            directionIndicator.transform.position = robotTransform.position + Vector3.up * 0.3f;
            directionIndicator.transform.rotation = Quaternion.LookRotation(robotTransform.forward, Vector3.up);
        }
    }

    void UpdateStatus()
    {
        if (statusText != null)
        {
            string status = $"Status: {(isMoving ? "MOVING" : "STOPPED")}\n";
            status += $"Linear: {currentLinearVelocity:F2} m/s\n";
            status += $"Angular: {currentAngularVelocity:F2} deg/s";
            statusText.text = status;
        }
    }

    public void SetLinearVelocity(float velocity)
    {
        currentLinearVelocity = Mathf.Clamp(velocity, -maxLinearVelocity, maxLinearVelocity);
        if (linearVelocitySlider != null)
        {
            linearVelocitySlider.value = currentLinearVelocity;
        }
    }

    public void SetAngularVelocity(float velocity)
    {
        currentAngularVelocity = Mathf.Clamp(velocity, -maxAngularVelocity, maxAngularVelocity);
        if (angularVelocitySlider != null)
        {
            angularVelocitySlider.value = currentAngularVelocity;
        }
    }

    void OnLinearVelocityChanged(float value)
    {
        currentLinearVelocity = value;
    }

    void OnAngularVelocityChanged(float value)
    {
        currentAngularVelocity = value;
    }

    public void EmergencyStop()
    {
        SetLinearVelocity(0f);
        SetAngularVelocity(0f);
        if (statusText != null)
        {
            statusText.text += "\nEMERGENCY STOP ACTIVATED";
        }
    }

    public void ResetControl()
    {
        SetLinearVelocity(0f);
        SetAngularVelocity(0f);
    }
}
```

## Summary Examples

### Complete Digital Twin Visualization System

Here's a complete example that combines multiple visualization techniques:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class DigitalTwinVisualizationSystem : MonoBehaviour
{
    [Header("Robot Components")]
    public GameObject robotModel;
    public RobotStateVisualizer stateVisualizer;
    public LIDARVisualization lidarViz;
    public CameraFeedVisualizer cameraViz;
    public OccupancyGridVisualizer gridViz;
    public PathVisualization pathViz;

    [Header("Interface Components")]
    public ControlInterfaceVisualizer controlViz;
    public bool enableRealtimeUpdates = true;

    [Header("Performance Settings")]
    public float updateInterval = 0.1f;
    private float lastUpdate;

    void Start()
    {
        InitializeSystem();
    }

    void InitializeSystem()
    {
        // Initialize all visualization components
        if (stateVisualizer == null)
            stateVisualizer = robotModel.GetComponent<RobotStateVisualizer>();

        if (lidarViz == null)
            lidarViz = GetComponent<LIDARVisualization>();

        if (cameraViz == null)
            cameraViz = GetComponent<CameraFeedVisualizer>();

        if (gridViz == null)
            gridViz = GetComponent<OccupancyGridVisualizer>();

        if (pathViz == null)
            pathViz = GetComponent<PathVisualization>();

        if (controlViz == null)
            controlViz = GetComponent<ControlInterfaceVisualizer>();

        lastUpdate = Time.time;
    }

    void Update()
    {
        if (enableRealtimeUpdates && Time.time - lastUpdate >= updateInterval)
        {
            UpdateSystem();
            lastUpdate = Time.time;
        }
    }

    void UpdateSystem()
    {
        // Update all visualization components
        UpdateRobotState();
        UpdateSensorData();
        UpdateEnvironment();
        UpdatePaths();
    }

    void UpdateRobotState()
    {
        if (stateVisualizer != null)
        {
            // Update based on robot state from ROS or other sources
            // This would typically come from ROS messages
        }
    }

    void UpdateSensorData()
    {
        if (lidarViz != null)
        {
            // Update LIDAR visualization with new data
            // This would come from ROS LIDAR messages
        }

        if (cameraViz != null)
        {
            // Update camera feed visualization
            // This would come from ROS image messages
        }
    }

    void UpdateEnvironment()
    {
        if (gridViz != null)
        {
            // Update occupancy grid visualization
            // This would come from SLAM or mapping algorithms
        }
    }

    void UpdatePaths()
    {
        if (pathViz != null)
        {
            // Update path visualization
            // This would come from path planning algorithms
        }
    }

    // Public methods for external control
    public void SetRobotOperational()
    {
        if (stateVisualizer != null)
            stateVisualizer.SetRobotState(RobotState.IDLE);
    }

    public void SetRobotMoving()
    {
        if (stateVisualizer != null)
            stateVisualizer.SetRobotState(RobotState.MOVING);
    }

    public void UpdateLIDARData(float[] ranges)
    {
        if (lidarViz != null)
            lidarViz.UpdateLIDARData(ranges);
    }

    public void EmergencyStop()
    {
        if (controlViz != null)
            controlViz.EmergencyStop();

        if (stateVisualizer != null)
            stateVisualizer.SetRobotState(RobotState.ERROR);
    }
}
```

## Best Practices for Visualization

### Performance Optimization
- Use object pooling for frequently created/destroyed objects
- Implement Level of Detail (LOD) for complex visualizations
- Optimize texture sizes and use compression
- Limit the number of simultaneous visual effects

### Visual Clarity
- Use consistent color schemes across visualizations
- Maintain appropriate contrast ratios
- Provide clear visual hierarchies
- Use visual cues to direct attention appropriately

### User Experience
- Provide intuitive controls and feedback
- Maintain consistent interaction patterns
- Offer customization options for different users
- Include help and guidance systems

## Summary

These visualization examples demonstrate various techniques for creating effective digital twin visualizations in Unity. From basic robot state visualization to complex sensor data rendering, each example provides a foundation that can be adapted to specific use cases. The key to effective visualization is balancing information richness with visual clarity while maintaining real-time performance.

## Exercises

1. Implement a complete digital twin visualization system using the provided examples
2. Create custom visualizations for a specific robot type
3. Add real-time data integration with ROS messages
4. Implement advanced visualization techniques like AR overlays
5. Optimize visualization performance for complex scenes
// Constants
const MAX_DISTANCE = 4.0;  // Maximum distance in meters for visualization

// DOM elements
const distanceValue = document.getElementById('distance-value');
const distanceBar = document.getElementById('distance-bar');
const obstacleStatus = document.getElementById('obstacle-status');
const obstacleIndicator = document.getElementById('obstacle-indicator');
const lastUpdate = document.getElementById('last-update');

// Function to fetch sensor data
function fetchSensorData() {
    fetch('/api/sensor_data')
        .then(response => response.json())
        .then(data => {
            // Update ultrasonic sensor display
            distanceValue.textContent = data.distance.toFixed(2);
            
            // Calculate bar width as percentage (inverse: closer = longer bar)
            const percentage = Math.max(0, Math.min(100, 100 - (data.distance / MAX_DISTANCE * 100)));
            distanceBar.style.width = `${percentage}%`;
            
            // Update IR sensor display
            obstacleStatus.textContent = data.obstacle ? "OBSTACLE DETECTED" : "NO OBSTACLE";
            
            if (data.obstacle) {
                obstacleIndicator.classList.add('obstacle-detected');
                obstacleIndicator.classList.remove('no-obstacle');
            } else {
                obstacleIndicator.classList.add('no-obstacle');
                obstacleIndicator.classList.remove('obstacle-detected');
            }
            
            // Update timestamp
            const date = new Date(data.timestamp * 1000);
            lastUpdate.textContent = date.toLocaleTimeString();
        })
        .catch(error => {
            console.error('Error fetching sensor data:', error);
        });
}

// Initial fetch
fetchSensorData();

// Fetch data periodically
setInterval(fetchSensorData, 200);  // Fetch every 200ms

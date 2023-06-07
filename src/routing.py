import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# Need to implement energy instead
maxrange = 270 * 1000  # Max range by safety factor
speed = 27
inspection_time = 30 * 60

# Generate sample data
np.random.seed(1)
X = np.random.randn(200, 2) * 30000  # Replace with your own dataset

# Calculate distances from origin
distances = np.linalg.norm(X, axis=1)

visited = []
left = X.copy()
distancesleft = distances.copy()
route = []

# Define the dynamic cluster size threshold
while len(left) > 0:
    dist = 0
    start = (0, 0)
    return_back = False
    closestindex = np.argmin(distancesleft)
    trip = [start]
    dest_dist = distancesleft[closestindex]
    distancesleft = np.delete(distancesleft, closestindex)
    while not return_back:
        dest = left[closestindex]
        left = np.delete(left, closestindex, axis=0)
        dist += dest_dist
        visited.append(dest)
        trip.append(dest)
        start = dest
        dist += inspection_time * speed
        if len(left) == 0:
            dist += np.linalg.norm(dest)
            break
        new_dist = np.linalg.norm(left - start, axis=1)
        closestindex = np.argmin(new_dist)
        pot_dist = dist + new_dist[closestindex] + np.linalg.norm(left[closestindex]) + inspection_time * speed
        if pot_dist > maxrange or len(left) == 0:
            dist += np.linalg.norm(dest)
            return_back = True
        else:
            dest_dist = new_dist[closestindex]
            distancesleft = np.delete(distancesleft, closestindex)
    if dist > maxrange:
        print('Range exceeds maximum')
    trip.append((0, 0))
    x = [i[0] for i in trip]
    y = [i[1] for i in trip]
    plt.scatter(x, y)
    plt.plot(x, y)
    route.append((trip, dist))
print(len(route))
plt.show()







#
#
# four = X[distances <= threshold]
# four_size = len(four)//4
# kmeans4 = KMeans(n_clusters=four_size)
# kmeans4.fit(four)
# labels4 = kmeans4.labels_
#
# plt.scatter(four[:, 0], four[:, 1], c=labels4, cmap='viridis')
# plt.show()
#
# # Determine the cluster size for each point
# cluster_sizes = np.where(distances <= threshold, 4, 1)
#
#
# # Modify k-means algorithm with custom cluster sizes
# kmeans = KMeans(n_clusters=len(cluster_sizes))
# kmeans.fit(X, sample_weight=cluster_sizes)

# # Get cluster labels and centroids
# labels = kmeans.labels_
# centroids = kmeans.cluster_centers_
#
# # Plot the clusters
# plt.scatter(X[:, 0], X[:, 1], c=labels, cmap='viridis')
# # plt.scatter(centroids[:, 0], centroids[:, 1], marker='x', color='red')
# plt.title('Modified k-means Clustering')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.legend()
# plt.show()
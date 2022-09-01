# CornerDetection-Lidar
To detect corners, we know that there are only four corners in the map, and each is 90 degrees. An easy approach that works well enough for this simple environment is to take 3 points separated by at least a few degrees (for example, from 9 consecutive points, you can take the first, fourth and ninth point). Then calculate the angle between these two points, and if it’s close to 90 degrees, you can assume that it’s a corner.

First we define a function in order to get the angles:

```sh
def getAngle(self, a, b, c):
        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        return ang + 360 if ang < 0 else ang
```
Then for corner detection:

```sh
array_t = []
        for key, range in enumerate(points, start=1):
            if key < 352:
                a = points[key]
                b = points[key+3]
                c = points[key+8]
                calculated_angle = self.getAngle(a, b, c)
                calculated_angle = calculated_angle - 180
                if calculated_angle > 75:
                    array_t.append(a)
                    array_t.append(b)
                    array_t.append(c)
                    points_np1 = np.array(array_t)
                    fig, ax = plt.subplots()
                    fig.suptitle('Corner Detection')
                    ax.scatter(points_np1[:,0], points_np1[:,1])
                    ax.set_title('Original points')
                    plt.show()
```

To detect lines, a simple approach is to use a polynomial fit of first degree, i.e., a linear fit (https://numpy.org/doc/stable/reference/generated/numpy.polyfit.html) to groups of points (for example, groups of 5 to 10 consecutive points). If two consecutive groups give you a fit that have almost the same parameters, then you can assume that both groups belong to the same line. Continue until you find a group that gives different parameters.


```sh
eight_split = np.array_split(points, 45)
        pd1_array_m = []
        pd1_array_b = []
        pd2_array_m = []
        pd2_array_b = []
        for key, range in enumerate(eight_split):
            if key % 2 == 0:
                xs1 = range[:,0]
                ys1 = range[:,1]
                pf1 = np.polyfit(np.array(xs1), np.array(ys1), 1)
                pd1 = np.poly1d(pf1)
                pd1_array_m.append(pd1[1])
                pd1_array_b.append(pd1[0])

            else:
                xs2 = range[:,0]
                ys2 = range[:,1]
                pf2 = np.polyfit(np.array(xs2), np.array(ys2), 1)
                pd2 = np.poly1d(pf2)
                pd2_array_m.append(pd2[1])
                pd2_array_b.append(pd2[0])

        count = 0
        while(count < 22):
            if 0 < abs(pd1_array_m[count]) < 1 and 2 < abs(pd2_array_b[count]) < 3:
                print("in the same line")
            else:
                print("not in the same line")
            count = count+1
        print("---------------end line")
```

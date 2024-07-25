import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

b = bagreader('/home/arvinder/RSN/src/Gps_driver/Analysis/rosbag_data/ISEC_moving.bag')

gpsdata= b.message_by_topic(topic="/gpstopic")
gpsdataread = pd.read_csv(gpsdata)
gpsdf = pd.DataFrame(gpsdataread, columns=[ "latitude.data", "longitude.data", "altitude.data", "utm_easting.data", "utm_northing.data", "fix_quality.data"]).astype(float)

df1 = pd.DataFrame(columns=["easting_fix","northing_fix"])
df2 = pd.DataFrame(columns=["easting_float","northing_float"])
df3 = pd.DataFrame(columns=["easting_single","northing_single"])


for ind in gpsdf.index:
    if gpsdf["fix_quality.data"][ind] == 4:
        df1.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]

    elif gpsdf["fix_quality.data"][ind] == 5:
        df2.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]
    
    elif gpsdf["fix_quality.data"][ind] == 1:  
        df3.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]


plt.scatter(df2["easting_float"], df2["northing_float"],label= "UTM(Easting,Northing) for solution status FLOAT", c ='blue')
plt.scatter(df1["easting_fix"], df1["northing_fix"],label= "UTM(Easting,Northing) for solution status FIX", c='green')
plt.scatter(df3["easting_single"], df3["northing_single"],label= "UTM(Easting,Northing) for solution status SINGLE", c ='black')

X1  = gpsdf["utm_easting.data"][:158].values.reshape(-1, 1)
Y1 = gpsdf["utm_northing.data"][:158].values.reshape(-1, 1)
linear_regressor1 = LinearRegression()
linear_regressor1.fit(X1, Y1)  
Y_pred1 = linear_regressor1.predict(X1)
plt.plot(X1, Y_pred1, color = "r", label= "Regression Line")
max_dist_float1 = max(abs(Y1[:145] - Y_pred1[:145]).max(), abs(Y1[152:158] - Y_pred1[152:158]).max())
max_dist_fix1 = abs(Y1[146:151] - Y_pred1[146:151]).max()
mean_dist1 = abs(Y1 - Y_pred1).mean()
r1 = r2_score(Y1, Y_pred1)

X2  = gpsdf["utm_easting.data"][159:647].values.reshape(-1, 1)
Y2 = gpsdf["utm_northing.data"][159:647].values.reshape(-1, 1)
linear_regressor2 = LinearRegression()
linear_regressor2.fit(X2, Y2)  
Y_pred2 = linear_regressor2.predict(X2)
plt.plot(X2, Y_pred2, color = "r")
max_dist_float2 = max(abs(Y2[159-159:187-159] - Y_pred2[159-159:187-159]).max(), abs(Y2[190-159:578-159] - Y_pred2[190-159:578-159]).max(), abs(Y2[581-159:647-159] - Y_pred2[581-159:647-159]).max())
max_dist_fix2 = abs(Y2[188-159:189-159] - Y_pred2[188-159:189-159]).max()
r2 = r2_score(Y2, Y_pred2)

X3  = gpsdf["utm_easting.data"][648:770].values.reshape(-1, 1)
Y3 = gpsdf["utm_northing.data"][648:770].values.reshape(-1, 1)
linear_regressor3 = LinearRegression()
linear_regressor3.fit(X3, Y3)  
Y_pred3 = linear_regressor3.predict(X3)
plt.plot(X3, Y_pred3, color = "r")
max_dist_float3 = max(abs(Y3[648-648:665-648] - Y_pred3[648-648:665-648]).max(), abs(Y3[670-648:770-648] - Y_pred3[670-648:770-648]).max())
max_dist_fix3 = abs(Y3[666-648:669-648] - Y_pred3[666-648:669-648]).max()
r3 = r2_score(Y3, Y_pred3)

X4  = gpsdf["utm_easting.data"][771:].values.reshape(-1, 1)
Y4 = gpsdf["utm_northing.data"][771:].values.reshape(-1, 1)
linear_regressor4 = LinearRegression()
linear_regressor4.fit(X4, Y4)  
Y_pred4 = linear_regressor4.predict(X4)
plt.plot(X4, Y_pred4, color = "r")
max_dist_float4 = abs(Y4 - Y_pred4).max()
r4 = r2_score(Y4, Y_pred4)

print("Maximum Distance(error) from best fit line for solution status FIX: %fm" %min(max_dist_fix1,max_dist_fix2,max_dist_fix3))
print("Maximum Distance(error) from best fit line for solution status FLOAT: %fm" %max(max_dist_float1, max_dist_float2, max_dist_float3, max_dist_float4))
print("Coefficient of determination: %f" %min(r1,r2,r3,r4))

plt.legend(loc="upper left")
plt.title("UTM Easting vs UTM Northing with obstructions(Moving)")
plt.xlabel("UTM Easting(m)")
plt.ylabel("UTM Northing(m)")
plt.grid(True)
plt.show()





import numpy as np
from sklearn.linear_model import LinearRegression, Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import Pipeline

theta = np.array([1,2])

# x = np.random.rand(3,2)
x = np.array([[0,0],[0,0],[1,1],[1,2]])
# y = x @ theta # @ is matrix multiplication
y = x@theta + .5*np.random.rand(1, x.shape[0])[0]

print ( " X ␣ = ␣ " ,x )
print ( " y ␣ = ␣ " ,y )

reg = LinearRegression()
reg.fit(x,y)
etheta = reg.coef_ # data's inclination

print ( " etheta ␣ = ␣ " , etheta )

reg2 = Ridge(alpha=1e-1)
reg2.fit(x,y)
etheta = reg2.coef_ # estimated theta

print(" etheta2 ␣ = ␣ " ,etheta)

model = Pipeline([('poly', PolynomialFeatures(degree=3)), ('linear', LinearRegression(fit_intercept=False))])

x = np.arange(5)
y = 3 - 2 * x + x ** 2 - x ** 3
model = model.fit(x[:,np.newaxis],y)
print(model.named_steps['linear'].coef_)

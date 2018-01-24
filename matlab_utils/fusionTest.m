clear all

filterVector(1) = kalmanFilter();
initialize(filterVector(1), [0 0 0 0 0 0]', 0.15);
filterVector(2) = kalmanFilter();
initialize(filterVector(2), [5 0 0 0 0 0]', 0.15);

predict(filterVector(1));
updateSelf(filterVector(1), [0 1 0]', 1e2*eye(3));

predict(filterVector(2));
updateSelf(filterVector(2), [5 1 0]', 1e2*eye(3));

[xrel, Prel] = propagateRelativeCovariance(filterVector(1), [100 0 0]', 1e-3*eye(3));
xrel
fuseRelative(filterVector(2), xrel, Prel);

Prel
filterVector(1).xPosterior
%filterVector(1).Pposterior

filterVector(2).xPosterior
%filterVector(2).Pposterior
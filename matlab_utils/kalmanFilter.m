classdef kalmanFilter < handle
    properties
        A; H; P; Q; R;
        xPrior;  xPosterior;
        Pprior;  Pposterior;
    end
    methods
        function obj = kalmanFilter()
            obj.A = [];
            obj.H = [];
            obj.P = 1*eye(3);
            obj.xPrior = [];
            obj.xPosterior = [];
            obj.Pprior = [];
            obj.Pposterior = [];
            
        end
        function initialize(obj, x, dt, posNoise, velNoise, accNoise)
            obj.Pposterior = obj.P;
            obj.xPosterior = x;
            
            obj.A = [eye(3)];
            obj.H = [1 0 0; 0 1 0; 0 0 1];
            
            obj.Q = [posNoise*eye(3)];
        end
        
        function predict(obj)
            obj.xPrior = obj.A * obj.xPosterior;
            obj.Pprior = obj.Pposterior + obj.Q;
        end
        
        function updateSelf(obj, z, R)
            yresidual = z - obj.H * obj.xPrior;
            S = obj.H * obj.Pprior * obj.H' + R;
            K = obj.Pprior * obj.H'/S;
    
            obj.xPosterior = obj.xPrior + K * (yresidual); 
            obj.Pposterior = (eye(3) - K * obj.H)*obj.Pprior;
        end
        
        function [xrel, Prel] = propagateRelativeCovariance(obj, z, R)
            Hrel = eye(3);
            xrel = obj.xPrior + z;
            Prel = Hrel * obj.Pposterior * Hrel' + R;
        end
        
        function fuseRelative(obj, xrel, Prel)
            Pself = obj.Pposterior;
            
            %Prel = [Prel zeros(3,3) ; zeros(3) eye(3)];
            [omega, ppost] = covarianceIntersection(Pself, Prel, eye(3));
            obj.Pposterior = ppost;
            omega
            obj.xPosterior = obj.Pposterior * ((omega * inv(Pself) * obj.xPosterior) + ((1-omega) * inv(Prel) * xrel));
        end
    end
end


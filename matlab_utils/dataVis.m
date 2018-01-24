figure(2)
subplot(3,1,1)
plot(1:190, colocFiltered{1}(:,1), 1:190, groundTruth{1}(:,1), 1:190, colocRaw{1}(:,1))
legend('Filtered', 'Ground Truth', 'Unfiltered');
title('X');
subplot(3,1,2)
plot(1:190, colocFiltered{1}(:,2), 1:190, groundTruth{1}(:,2), 1:190, colocRaw{1}(:,2))
legend('Filtered', 'Ground Truth', 'Unfiltered');
title('Y');
subplot(3,1,3)
plot(1:190, colocFiltered{1}(:,3), 1:190, groundTruth{1}(:,3), 1:190, colocRaw{1}(:,3))
legend('Filtered', 'Ground Truth', 'Unfiltered');
title('Z');
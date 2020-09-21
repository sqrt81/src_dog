% 把结果打印到文件中。
% 需要的内容包括：正向运动学、关节Jacobian矩阵、动力学矩阵H、C、G
clear;

file_id = fopen("result.txt", 'w');

LegTransform;
foot_pos = T1 * T2 * T3 * FootLocalPos;
fprintf(file_id, 'foot pos:\n');
fprintf(file_id, '%s;\n', foot_pos(1:3));
fprintf(file_id, '\njacobian:\n');
fprintf(file_id, '%s; \t%s; \t%s;\n', jacobian(foot_pos(1:3), [q1 q2 q3]));

DynamicsMatrix;
fprintf(file_id, '\nH matrix:\n');

for i = [1 2 3]
    for j = [1 2 3]
        fprintf(file_id, 'H(%d, %d): %s\n\n', i, j, H(i, j));
    end
end

fprintf(file_id, '\nC matrix:\n');

for i = [1 2 3]
    for j = [1 2 3]
        for k = [vq1 vq2 vq3]
            fprintf(file_id, 'dC(%d, %d)/d%s: %s\n\n', i, j, k, ...
                diff(C(i, j), k));
        end
    end
end

fprintf(file_id, '\nG matrix:\n');

for i = [1 2 3]
    for j = [1 2 3]
        fprintf(file_id, 'G(%d, %d): %s\n\n', i, j, G(i, j));
    end
end

fclose(file_id);

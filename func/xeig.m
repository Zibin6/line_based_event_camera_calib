% eig -- the eigenvalue is sorted特征指已排序
function [V,D] = xeig(A)

	[V,D] = eig(A);
%     V的列向量是A的特征向量构成，D是A 的特征值
	if ~issorted(diag(D))
		[V,D] = eig(A);
		[D,I] = sort(diag(D));
		V = V(:, I);
	end

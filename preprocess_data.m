function X = preprocess_data(X,K)


T     = size(X,1);
cx    = size(X,2);
count = 0;

for i = 1 : T
    if(rem(i-1,K)==0)
        count = count+1;
    end
end

L    = count;
Xnew = zeros(L,cx);

for i = 1 : L
    Xnew(i,:) = X(1+(i-1)*K,:);
end

X = Xnew;
end
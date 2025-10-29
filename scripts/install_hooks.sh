cd ..

rm -rf .git/hooks
mkdir -p .git/hooks

cp -r hooks/ .git/hooks/
chmod +x .git/hooks/*

echo "Git hooks installed successfully."

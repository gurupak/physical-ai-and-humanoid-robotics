const fs = require('fs');

const filePath = 'docs/chapter-3-isaac-ai-brain/final-documentation-package.md';
let content = fs.readFileSync(filePath, 'utf8');

// Replace single curly braces with double curly braces in Python code blocks
// This escapes them for MDX
content = content.replace(/\{config\./g, '{{config.');
content = content.replace(/\{(\w+)\}/g, '{{$1}}');

fs.writeFileSync(filePath, content, 'utf8');
console.log('Fixed curly braces in', filePath);

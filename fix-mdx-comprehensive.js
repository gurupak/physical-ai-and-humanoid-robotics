const fs = require('fs');

function fixMDXFile(filename) {
  let content = fs.readFileSync(filename, 'utf8');

  // Track the changes
  let modified = false;
  const originalContent = content;

  // 1. Fix Python f-strings that contain curly braces by escaping them
  // This is crucial for line 533 and similar lines
  content = content.replace(/f"""([^"]|"(?!""))*"""/gs, (match) => {
    // Escape { and } in f-string expressions by wrapping entire block
    if (match.includes('{') && match.includes('}')) {
      modified = true;
      // Convert to triple backtick block if not already
      if (!match.startsWith('```')) {
        return '```python\n' + match.replace(/f"""/, 'f"""').replace(/"""/, '"""') + '\n```';
      }
    }
    return match;
  });

  // 2. Fix inline Python f-strings
  content = content.replace(/^([^`]*\w+\s*=\s*)f"([^"`]*)"([^`]*$)/gm, (match, prefix, fstring, suffix) => {
    if (fstring.includes('{') && fstring.includes('}')) {
      modified = true;
      // Check if we're in a code block
      const lines = content.split('\n');
      const lineIndex = lines.findIndex(line => line.includes(match));
      if (lineIndex !== -1) {
        // Count how many triple backticks before
        const beforeText = lines.slice(0, lineIndex).join('\n');
        const codeBlockCount = (beforeText.match(/```/g) || []).length;
        if (codeBlockCount % 2 === 0) {
          // Not in a code block, need to put in one
          return prefix + "f'" + fstring + "'" + suffix;
        }
      }
    }
    return match;
  });

  // 3. Fix unescaped curly braces in bash scripts
  content = content.replace(/(\$\{[^}]*\})\(PASS_COUNT\+\+\)/g, (match) => {
    modified = true;
    return match.replace('((', '(').replace('))', ')');
  });

  // 4. Fix corrupted backtick patterns
  content = content.replace(/```(\w+) title="([^"]+)```/g, (match, lang, title) => {
    modified = true;
    return '```' + lang + ' title="' + title + '"';
  });

  // 5. Remove or fix @title syntax that's causing issues
  content = content.replace(/title="([^"]+)"\s*(\w+)\s*(=\s*\d{1,3})/g, (match, title, rest) => {
    modified = true;
    return 'title="' + title + '"';
  });

  // 6. Fix any remaining corrupted escape sequences
  content = content.replace(/\{\\em}\s*fontspec\s*\{[^}]*/g, '');

  // 7. Properly close code blocks that aren't closed
  const lines = content.split('\n');
  let inCodeBlock = false;
  let codeBlockStack = [];

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    if (line.startsWith('```')) {
      const trimmed = line.trim();
      if (codeBlockStack.length === 0 || codeBlockStack[codeBlockStack.length - 1] !== trimmed) {
        codeBlockStack.push(trimmed);
        inCodeBlock = true;
      } else {
        codeBlockStack.pop();
        inCodeBlock = codeBlockStack.length > 0;
      }
    }
  }

  // Add closing backticks if needed
  if (codeBlockStack.length > 0) {
    content += '\n' + codeBlockStack.reverse().join('\n');
    modified = true;
  }

  // Special fix for the problematic f-string in final-documentation-package.md
  if (filename.includes('final-documentation-package')) {
    content = content.replace(
      /(index_content = f""")((.|\n)*?)(""")/s,
      (match, prefix, middle, suffix) => {
        modified = true;
        // Wrap in code block
        return '```python\n' + prefix + middle + suffix + '\n```';
      }
    );
  }

  if (modified) {
    fs.writeFileSync(filename, content, 'utf8');
    console.log(`Fixed ${filename}`);
  } else {
    console.log(`No changes needed for ${filename}`);
  }

  return modified;
}

// Process all files
const files = [
  'docs/chapter-3-isaac-ai-brain/dynamic-gait-transitions.md',
  'docs/chapter-3-isaac-ai-brain/final-documentation-package.md',
  'docs/chapter-3-isaac-ai-brain/walking-gait-integration.md'
];

let totalFixed = 0;
files.forEach(file => {
  if (fs.existsSync(file)) {
    if (fixMDXFile(file)) {
      totalFixed++;
    }
  }
});

console.log(`\nFixed ${totalFixed} files`);
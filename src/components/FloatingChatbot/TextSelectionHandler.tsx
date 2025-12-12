import { useEffect } from 'react';

interface TextSelectionHandlerProps {
  onTextSelected: (text: string) => void;
}

export default function TextSelectionHandler({ onTextSelected }: TextSelectionHandlerProps): null {
  useEffect(() => {
    const handleContextMenu = (e: MouseEvent) => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 0) {
        // Check if the selection is within the documentation content
        const target = e.target as HTMLElement;
        const isInDocContent = target.closest('article') || target.closest('.markdown');

        if (isInDocContent) {
          e.preventDefault();

          // Create custom context menu
          const contextMenu = document.createElement('div');
          contextMenu.className = 'chatbot-context-menu';
          contextMenu.innerHTML = `
            <div class="context-menu-item">
              <span class="context-menu-icon">🤖</span>
              <span>Ask AI about this text</span>
            </div>
          `;

          contextMenu.style.position = 'fixed';
          contextMenu.style.left = `${e.clientX}px`;
          contextMenu.style.top = `${e.clientY}px`;
          contextMenu.style.zIndex = '10000';

          contextMenu.addEventListener('click', () => {
            onTextSelected(selectedText);
            document.body.removeChild(contextMenu);
            selection?.removeAllRanges();
          });

          // Remove menu when clicking elsewhere
          const removeMenu = () => {
            if (document.body.contains(contextMenu)) {
              document.body.removeChild(contextMenu);
            }
            document.removeEventListener('click', removeMenu);
          };

          setTimeout(() => {
            document.addEventListener('click', removeMenu);
          }, 100);

          document.body.appendChild(contextMenu);
        }
      }
    };

    document.addEventListener('contextmenu', handleContextMenu);

    return () => {
      document.removeEventListener('contextmenu', handleContextMenu);
    };
  }, [onTextSelected]);

  return null;
}

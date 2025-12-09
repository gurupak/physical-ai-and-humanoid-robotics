import React, { useState } from 'react';
import styles from './Quiz.module.css';

export interface QuizProps {
  question: string;
  options: string[];
  correctAnswer: number;
  explanation: string;
  type?: 'multiple-choice' | 'true-false' | 'code';
  codeSnippet?: string;
  difficulty?: 'easy' | 'medium' | 'hard';
}

const Quiz: React.FC<QuizProps> = ({
  question,
  options,
  correctAnswer,
  explanation,
  type = 'multiple-choice',
  codeSnippet,
  difficulty = 'medium',
}) => {
  const [selectedAnswer, setSelectedAnswer] = useState<number | null>(null);
  const [showFeedback, setShowFeedback] = useState(false);

  const handleAnswerSelect = (index: number) => {
    if (showFeedback) return; // Prevent changing answer after submission
    setSelectedAnswer(index);
    setShowFeedback(true);
  };

  const handleReset = () => {
    setSelectedAnswer(null);
    setShowFeedback(false);
  };

  const isCorrect = selectedAnswer === correctAnswer;

  return (
    <div className={`quiz-container ${styles.quizContainer}`} data-difficulty={difficulty}>
      <div className={styles.quizHeader}>
        <span className={styles.quizLabel}>Quiz</span>
        {difficulty && (
          <span className={`${styles.difficultyBadge} ${styles[difficulty]}`}>
            {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}
          </span>
        )}
      </div>

      <div className={styles.question}>{question}</div>

      {codeSnippet && (
        <pre className={styles.codeSnippet}>
          <code>{codeSnippet}</code>
        </pre>
      )}

      <div className={styles.options}>
        {options.map((option, index) => (
          <button
            key={index}
            className={`${styles.option} ${
              selectedAnswer === index ? styles.selected : ''
            } ${
              showFeedback && index === correctAnswer
                ? styles.correct
                : showFeedback && selectedAnswer === index
                ? styles.incorrect
                : ''
            }`}
            onClick={() => handleAnswerSelect(index)}
            disabled={showFeedback}
            aria-pressed={selectedAnswer === index}
          >
            <span className={styles.optionLetter}>
              {String.fromCharCode(65 + index)}.
            </span>
            <span className={styles.optionText}>{option}</span>
            {showFeedback && index === correctAnswer && (
              <span className={styles.checkmark}>✓</span>
            )}
            {showFeedback && selectedAnswer === index && index !== correctAnswer && (
              <span className={styles.cross}>✗</span>
            )}
          </button>
        ))}
      </div>

      {showFeedback && (
        <div className={`${styles.feedback} ${isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}>
          <div className={styles.feedbackHeader}>
            {isCorrect ? (
              <>
                <span className={styles.feedbackIcon}>✓</span>
                <strong>Correct!</strong>
              </>
            ) : (
              <>
                <span className={styles.feedbackIcon}>✗</span>
                <strong>Not quite right</strong>
              </>
            )}
          </div>
          <div className={styles.explanation}>{explanation}</div>
          <button className={styles.retryButton} onClick={handleReset}>
            Try Again
          </button>
        </div>
      )}
    </div>
  );
};

export default Quiz;

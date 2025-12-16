-- Add profile fields to user table
ALTER TABLE "user"
ADD COLUMN IF NOT EXISTS phone VARCHAR(20),
ADD COLUMN IF NOT EXISTS expertise_level VARCHAR(20) DEFAULT 'beginner',
ADD COLUMN IF NOT EXISTS first_name VARCHAR(100),
ADD COLUMN IF NOT EXISTS last_name VARCHAR(100);

-- Create index for faster lookups
CREATE INDEX IF NOT EXISTS idx_user_expertise_level ON "user"(expertise_level);

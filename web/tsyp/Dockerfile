FROM node:18

# Set the working directory
WORKDIR /app

# Copy the rest of the application files
COPY . .

# Install both production and development dependencies
RUN npm install

# Expose port 3000
EXPOSE 3000

# Start the application in development mode
CMD ["yarn", "dev"]
